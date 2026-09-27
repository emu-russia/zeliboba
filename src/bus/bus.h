// zeliboba - physical address bus.
//
// The bus owns RAM regions and MMIO devices, keeps a 4 KiB page table for the
// fast path, and records every MMIO access in a ring buffer (see TraceLog).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "bus/device.h"
#include "bus/trace.h"
#include "common/types.h"

namespace zlb {

struct MemRegion {
    std::string name;
    u32 base = 0;
    u32 size = 0;
    std::string note;
    bool mapped = false;
    bool readonly = false;
    std::vector<u8> data;

    /// When set, accesses go to this host buffer instead of `data`, which lets
    /// two address spaces share one piece of physical memory (the ARM/CMeP
    /// boot SRAM, for instance).
    u8* external = nullptr;

    u8* bytes() { return external ? external : data.data(); }
    const u8* bytes() const { return external ? external : data.data(); }

    u32 end() const { return base + size; }
    bool contains(u32 address, size_t length = 1) const {
        return mapped && address >= base && (static_cast<u64>(address) + length) <= (static_cast<u64>(base) + size);
    }
};

struct BusStats {
    u64 reads = 0;
    u64 writes = 0;
    u64 fetches = 0;
    u64 mmio = 0;
    u64 ram = 0;
    u64 unmapped = 0;
};

/// Development aid: set ZLB_WTRAP=<lo>-<hi> (hex) to log every write that lands
/// in that physical range together with the emulator function that performed it.
void bus_install_write_trap(u32 low, u32 high);
bool bus_write_trap_contains(u32 address);
/// Same idea for reads: `ZLB_RTRAP=<lo>-<hi>` traces who reads a physical range
/// (and what they get). Used to reconstruct parameter blocks a stage consumes
/// before it can be modelled - a write trap cannot answer "what did it read".
void bus_install_read_trap(u32 low, u32 high);
bool bus_read_trap_contains(u32 address);

/// Who is currently executing, for the access trace.
struct BusContext {
    std::string core = "none";
    u32 pc = 0;
};

class Bus {
public:
    Bus();
    ~Bus();

    Bus(const Bus&) = delete;
    Bus& operator=(const Bus&) = delete;

    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    /// Allocate a RAM region. Not mapped until `map_ram` is called (or `mapped`
    /// is set directly), mirroring the original test harness semantics.
    MemRegion& add_ram(const std::string& name, u32 size);
    MemRegion& add_ram(const std::string& name, u32 size, u32 base, const std::string& note);

    /// Map `size` bytes at `base` onto a host buffer owned by somebody else.
    /// Used for memory that two cores can see (the shared boot SRAM).
    MemRegion& add_ram_alias(const std::string& name, u32 base, u32 size, u8* host,
                             const std::string& note);

    /// Find or create a RAM region covering [address, address+size).
    MemRegion& ensure_ram(u32 address, size_t size, const std::string& tag_prefix = "auto");

    MemRegion* find_region(const std::string& name);
    MemRegion* region_at(u32 address, size_t size = 1);
    const MemRegion* region_at(u32 address, size_t size = 1) const;

    void add_device(std::unique_ptr<Device> device);
    Device* find_device(u32 address) const;
    int device_index(const Device* device) const;
    const std::vector<std::unique_ptr<Device>>& devices() const { return devices_; }
    std::vector<std::unique_ptr<Device>>& devices() { return devices_; }
    const std::vector<MemRegion>& regions() const { return regions_; }
    std::vector<MemRegion>& regions() { return regions_; }

    /// Rebuild the page table after changing the memory map.
    void rebuild_map();
    void reset_devices();
    void reset();

    // ------------------------------------------------------------------
    // Access
    // ------------------------------------------------------------------

    u8 read8(u32 address);
    u16 read16(u32 address);
    u32 read32(u32 address);
    u64 read64(u32 address);

    void write8(u32 address, u8 value);
    void write16(u32 address, u16 value);
    void write32(u32 address, u32 value);
    void write64(u32 address, u64 value);

    /// Instruction fetch (separately logged).
    u32 fetch32(u32 address);
    u16 fetch16(u32 address);
    u8 fetch8(u32 address);

    void read_bytes(u32 address, void* out, size_t length);
    void write_bytes(u32 address, const void* data, size_t length);

    /// Copy a blob into memory, auto-mapping RAM when required.
    bool load(u32 address, const void* data, size_t length, const std::string& tag = "load");

    /// Fill a buffer with a value.
    void memset_bytes(u32 address, u8 value, size_t length);

    bool is_mapped(u32 address, size_t size = 1) const;
    bool is_ram(u32 address, size_t size = 1) const;

    /// First unmapped address in [address, address+length), or 0 when fully mapped.
    u32 first_unmapped(u32 address, size_t length) const;

    // ------------------------------------------------------------------
    // Exclusive monitor (the SCU's global monitor)
    // ------------------------------------------------------------------
    //
    // LDREX/STREX are only exclusive with respect to *every* observer: a write by
    // another core to the monitored address must make the STREX fail. Keeping the
    // reservation inside one ArmCore (as this did) let two cores both succeed on
    // the same read-modify-write, so kernel_boot_loader's four-core barrier at
    // 0x4003B384 could lose an update and deadlock with the counter stuck at 3.
    struct ExclusiveReservation {
        bool valid = false;
        u32 address = 0;
        unsigned size = 0;
        int owner = -1;
        u32 asid = 0;
    };

    static constexpr int kMaxReservations = 8;

    /// Reserve [address, address+size) for `core` (LDREX).
    void mark_exclusive(u32 address, unsigned size, int core, u32 asid);
    /// Consume the reservation if `core` still holds one covering the range.
    bool take_exclusive(u32 address, unsigned size, int core, u32 asid);
    /// Drop `core`'s reservation (CLREX).
    void clear_exclusive_for(int core);
    /// Drop every reservation overlapping the range - called by all write paths.
    void clear_exclusive(u32 address, unsigned size);
    bool has_exclusive(int core) const;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    TraceLog trace;
    BusStats stats;
    BusContext context;

    /// Set when the last access missed everything.
    bool last_unmapped = false;

    /// When true, reads from unmapped memory return 0 (instead of all-ones) and
    /// writes are silently dropped. Boot code probing for hardware often needs
    /// this because the all-ones pattern looks like a live bus.
    bool unmapped_reads_zero = false;

    std::string describe_map() const;
    // Internal: used by the fast path.
    struct Page {
        u8* host = nullptr;       // page base inside a RAM region (kind == Ram)
        Device* device = nullptr; // MMIO device covering the page (kind == Mmio)
        u32 page_base = 0;
        u8 kind = 0;  // 0 = unmapped, 1 = ram, 2 = mmio, 3 = partial
    };

    static constexpr size_t kPageShift = 12;
    static constexpr size_t kPageSize = 1u << kPageShift;
    static constexpr size_t kPageCount = 1u << (32 - kPageShift);

private:
    void note(AccessKind kind, u32 address, unsigned size, u64 value, Device* device, bool unmapped);
    /// Record a fast-path RAM access when RAM tracing is enabled.
    void fast_trace(AccessKind kind, u32 address, unsigned size, u64 value);
    bool slow_read(u32 address, unsigned size, u64& out, Device*& device, bool fetch);
    bool slow_write(u32 address, unsigned size, u64 value, Device*& device);

    /// Development aid: logs writes inside the ZLB_WTRAP range.
    void note_write_trap(u32 address, unsigned size, u64 value);
    void note_read_trap(u32 address, unsigned size, u64 value);

    std::vector<MemRegion> regions_;
    std::vector<std::unique_ptr<Device>> devices_;
    std::vector<Page> pages_;
    ExclusiveReservation reservations_[kMaxReservations];
};

}  // namespace zlb
