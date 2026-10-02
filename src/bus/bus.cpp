#include "bus/bus.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#if defined(_MSC_VER)
#include <intrin.h>
#endif

#include "common/log.h"
#include "common/util.h"

namespace zlb {

namespace {

struct WriteTrap {
    bool enabled = false;
    u32 low = 0;
    u32 high = 0;
};

WriteTrap& write_trap() {
    static WriteTrap trap = [] {
        WriteTrap t;
        const char* text = std::getenv("ZLB_WTRAP");
        if (text && *text) {
            unsigned long long lo = 0, hi = 0;
            if (std::sscanf(text, "%llx-%llx", &lo, &hi) == 2) {
                t.enabled = true;
                t.low = static_cast<u32>(lo);
                t.high = static_cast<u32>(hi);
            }
        }
        return t;
    }();
    return trap;
}

WriteTrap& read_trap() {
    static WriteTrap trap = [] {
        WriteTrap t;
        const char* text = std::getenv("ZLB_RTRAP");
        if (text && *text) {
            unsigned long long lo = 0, hi = 0;
            if (std::sscanf(text, "%llx-%llx", &lo, &hi) == 2) {
                t.enabled = true;
                t.low = static_cast<u32>(lo);
                t.high = static_cast<u32>(hi);
            }
        }
        return t;
    }();
    return trap;
}

}  // namespace

void bus_install_write_trap(u32 low, u32 high) {
    WriteTrap& trap = write_trap();
    trap.enabled = true;
    trap.low = low;
    trap.high = high;
}

void bus_install_read_trap(u32 low, u32 high) {
    WriteTrap& trap = read_trap();
    trap.enabled = true;
    trap.low = low;
    trap.high = high;
}

bool bus_write_trap_contains(u32 address) {
    const WriteTrap& trap = write_trap();
    return trap.enabled && address >= trap.low && address < trap.high;
}

bool bus_read_trap_contains(u32 address) {
    const WriteTrap& trap = read_trap();
    return trap.enabled && address >= trap.low && address < trap.high;
}

/// Cheap "is a read trap configured" test for the hot read paths.
static bool read_trap_enabled() { return read_trap().enabled; }

void Bus::note_write_trap(u32 address, unsigned size, u64 value) {
    if (!bus_write_trap_contains(address)) return;
    const MemRegion* region = region_at(address, 1);
#if defined(_MSC_VER)
    void* caller = _ReturnAddress();
#elif defined(__GNUC__) || defined(__clang__)
    void* caller = __builtin_extract_return_addr(__builtin_return_address(0));
#else
    void* caller = nullptr;
#endif
    std::fprintf(stderr, "[wtrap] %-14s +0x%05X w%u = 0x%llX pc=%08X core=%s caller=%p\n",
                 region ? region->name.c_str() : "<none>", region ? address - region->base : address, size,
                 static_cast<unsigned long long>(value), context.pc, context.core,
                 caller);
}

void Bus::note_read_trap(u32 address, unsigned size, u64 value) {
    if (!bus_read_trap_contains(address)) return;
    const MemRegion* region = region_at(address, 1);
    std::fprintf(stderr, "[rtrap] %-14s +0x%05X r%u = 0x%llX pc=%08X core=%s\n",
                 region ? region->name.c_str() : "<none>", region ? address - region->base : address, size,
                 static_cast<unsigned long long>(value), context.pc, context.core);
}

// ---------------------------------------------------------------------------
// Exclusive monitor
// ---------------------------------------------------------------------------

namespace {
bool ranges_overlap(u32 a, unsigned a_size, u32 b, unsigned b_size) {
    const u64 a_end = static_cast<u64>(a) + a_size;
    const u64 b_end = static_cast<u64>(b) + b_size;
    return a < b_end && b < a_end;
}
}  // namespace

void Bus::mark_exclusive(u32 address, unsigned size, int core, u32 asid) {
    if (exclusive_trace) {
        ZLB_LOG_INFO("bus", "excl: mark core=%d asid=0x%X addr=0x%08X size=%u", core, asid, address,
                     size);
    }
    // One reservation per core, like the SCU's per-core monitor slots.
    ExclusiveReservation* slot = nullptr;
    for (ExclusiveReservation& reservation : reservations_) {
        if (reservation.valid && reservation.owner == core) {
            slot = &reservation;
            break;
        }
    }
    if (slot == nullptr) {
        for (ExclusiveReservation& reservation : reservations_) {
            if (!reservation.valid) {
                slot = &reservation;
                break;
            }
        }
    }
    if (slot == nullptr) slot = &reservations_[0];  // all slots busy: recycle the first
    slot->valid = true;
    slot->address = address;
    slot->size = size;
    slot->owner = core;
    slot->asid = asid;
}

bool Bus::take_exclusive(u32 address, unsigned size, int core, u32 asid) {
    for (ExclusiveReservation& reservation : reservations_) {
        if (!reservation.valid || reservation.owner != core) continue;
        const bool ok = reservation.address == address && reservation.size == size &&
                        reservation.asid == asid;
        if (exclusive_trace) {
            ZLB_LOG_INFO("bus",
                         "excl: take core=%d asid=0x%X addr=0x%08X size=%u -> %s "
                         "(held addr=0x%08X size=%u asid=0x%X)",
                         core, asid, address, size, ok ? "OK" : "FAIL", reservation.address,
                         reservation.size, reservation.asid);
        }
        reservation.valid = false;
        return ok;
    }
    if (exclusive_trace) {
        ZLB_LOG_INFO("bus", "excl: take core=%d asid=0x%X addr=0x%08X size=%u -> FAIL (no slot)",
                     core, asid, address, size);
    }
    return false;
}

void Bus::clear_exclusive_for(int core) {
    for (ExclusiveReservation& reservation : reservations_) {
        if (reservation.valid && reservation.owner == core) reservation.valid = false;
    }
}

void Bus::clear_exclusive(u32 address, unsigned size) {
    for (ExclusiveReservation& reservation : reservations_) {
        if (!reservation.valid) continue;
        if (ranges_overlap(reservation.address, reservation.size, address, size)) {
            if (exclusive_trace) {
                ZLB_LOG_INFO("bus",
                             "excl: clear by write addr=0x%08X size=%u dropped core=%d "
                             "reservation addr=0x%08X size=%u",
                             address, size, reservation.owner, reservation.address,
                             reservation.size);
            }
            reservation.valid = false;
        }
    }
}

bool Bus::has_exclusive(int core) const {
    for (const ExclusiveReservation& reservation : reservations_) {
        if (reservation.valid && reservation.owner == core) return true;
    }
    return false;
}

Bus::Bus() : trace(1u << 18) {
    pages_.resize(kPageCount);
    rebuild_map();
}

Bus::~Bus() = default;

// ---------------------------------------------------------------------------
// Regions
// ---------------------------------------------------------------------------

MemRegion& Bus::add_ram(const std::string& name, u32 size) {
    MemRegion region;
    region.name = name;
    region.size = size;
    region.base = 0;
    region.mapped = false;
    region.data.assign(size, 0);
    regions_.push_back(std::move(region));
    return regions_.back();
}

MemRegion& Bus::add_ram(const std::string& name, u32 size, u32 base, const std::string& note) {
    MemRegion& region = add_ram(name, size);
    region.base = base;
    region.mapped = true;
    region.note = note;
    rebuild_map();
    return region;
}

MemRegion* Bus::find_region(const std::string& name) {
    for (auto& region : regions_) {
        if (region.name == name) return &region;
    }
    return nullptr;
}

MemRegion* Bus::region_at(u32 address, size_t size) {
    for (auto& region : regions_) {
        if (region.contains(address, size)) return &region;
    }
    return nullptr;
}

const MemRegion* Bus::region_at(u32 address, size_t size) const {
    for (const auto& region : regions_) {
        if (region.contains(address, size)) return &region;
    }
    return nullptr;
}

MemRegion& Bus::add_ram_alias(const std::string& name, u32 base, u32 size, u8* host, const std::string& note) {
    MemRegion region;
    region.name = name;
    region.size = size;
    region.base = base;
    region.mapped = true;
    region.note = note;
    region.external = host;
    region.data.clear();
    regions_.push_back(std::move(region));
    rebuild_map();
    return regions_.back();
}

MemRegion& Bus::ensure_ram(u32 address, size_t size, const std::string& tag_prefix) {
    if (MemRegion* existing = region_at(address, size)) return *existing;

    const u32 page = 0x10000;
    u32 lo = address & ~(page - 1);
    u64 hi64 = (static_cast<u64>(address) + size + page - 1) & ~static_cast<u64>(page - 1);
    u32 hi = hi64 > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<u32>(hi64);
    u32 length = hi > lo ? (hi - lo) : page;

    MemRegion& region = add_ram(format("%s_auto@%08X", tag_prefix.c_str(), lo), length, lo, "auto-mapped");
    return region;
}

// ---------------------------------------------------------------------------
// Devices
// ---------------------------------------------------------------------------

void Bus::add_device(std::unique_ptr<Device> device) {
    devices_.push_back(std::move(device));
    rebuild_map();
}

Device* Bus::find_device(u32 address) const {
    // Smallest matching window wins, so a sub-device can override a large one.
    Device* best = nullptr;
    for (const auto& device : devices_) {
        if (!device->handles(address)) continue;
        if (best == nullptr || device->size() < best->size()) best = device.get();
    }
    return best;
}

int Bus::device_index(const Device* device) const {
    for (size_t i = 0; i < devices_.size(); ++i) {
        if (devices_[i].get() == device) return static_cast<int>(i);
    }
    return -1;
}

void Bus::rebuild_map() {
    for (auto& page : pages_) {
        page.host = nullptr;
        page.device = nullptr;
        page.kind = 0;
        page.page_base = 0;
    }

    for (auto& region : regions_) {
        if (!region.mapped || region.size == 0) continue;
        u32 first = region.base >> kPageShift;
        u32 last = (region.base + region.size - 1) >> kPageShift;
        for (u32 index = first; index <= last && index < kPageCount; ++index) {
            u64 page_start = static_cast<u64>(index) << kPageShift;
            if (page_start < region.base) {
                pages_[index].kind = 3;  // partial: slow path
                continue;
            }
            u64 offset = page_start - region.base;
            if (offset + kPageSize > region.size) {
                pages_[index].kind = 3;
                continue;
            }
            pages_[index].kind = 1;
            pages_[index].host = region.bytes() + offset;
            pages_[index].page_base = region.base;
        }
    }

    for (const auto& device : devices_) {
        u32 base = device->base();
        u32 size = device->size();
        if (size == 0) continue;
        u32 first = base >> kPageShift;
        u32 last = (base + size - 1) >> kPageShift;
        for (u32 index = first; index <= last && index < kPageCount; ++index) {
            u64 page_start = static_cast<u64>(index) << kPageShift;
            if (page_start < base || page_start + kPageSize > static_cast<u64>(base) + size) {
                if (pages_[index].kind != 1) pages_[index].kind = 3;
                continue;
            }
            pages_[index].kind = 2;
            pages_[index].device = device.get();
            pages_[index].host = nullptr;
            pages_[index].page_base = base;
        }
    }
}

void Bus::reset_devices() {
    for (auto& device : devices_) device->reset();
}

void Bus::reset() {
    context = BusContext{};
    for (auto& region : regions_) std::fill(region.bytes(), region.bytes() + region.size, 0);
    reset_devices();
    trace.clear();
    stats = BusStats{};
    last_unmapped = false;
}

// ---------------------------------------------------------------------------
// Access helpers
// ---------------------------------------------------------------------------

bool Bus::is_mapped(u32 address, size_t size) const {
    if (address + size > 0x100000000ull) return false;
    if (region_at(address, size)) return true;
    for (size_t offset = 0; offset < size; ++offset) {
        if (find_device(address + static_cast<u32>(offset))) return true;
    }
    return false;
}

bool Bus::is_ram(u32 address, size_t size) const { return region_at(address, size) != nullptr; }

u32 Bus::first_unmapped(u32 address, size_t length) const {
    for (size_t offset = 0; offset < length; ++offset) {
        u32 current = address + static_cast<u32>(offset);
        if (!is_mapped(current, 1)) return current;
    }
    return 0;
}

void Bus::fast_trace(AccessKind kind, u32 address, unsigned size, u64 value) {
    // The page-table fast paths bypass slow_read/slow_write, so a memory
    // watchpoint would never see a plain RAM access without this.
    if (!trace.trace_ram()) return;
    note(kind, address, size, value, nullptr, false);
}

void Bus::note(AccessKind kind, u32 address, unsigned size, u64 value, Device* device, bool unmapped) {
    AccessRecord record;
    record.kind = kind;
    record.size = static_cast<u8>(size);
    record.address = address;
    record.value = value;
    record.pc = context.pc;
    record.device = device ? device_index(device) : -1;
    record.unmapped = unmapped;
    record.sequence = trace.total();
    trace.push(record);
}

bool Bus::slow_read(u32 address, unsigned size, u64& out, Device*& device, bool fetch) {
    device = find_device(address);
    // The normal case: one device owns the whole access, so hand it the real
    // access size. Splitting a 32-bit register access into four byte accesses
    // would miss every word-addressed register file.
    if (device && device->handles(address + size - 1)) {
        stats.mmio++;
        const u64 value = device->read(address, size);
        out = value;
        note(fetch ? AccessKind::Fetch : AccessKind::Read, address, size, value, device, false);
        return true;
    }
    // Straddling the device window: decode byte by byte.
    if (device) {
        stats.mmio++;
        u64 value = 0;
        for (unsigned i = 0; i < size; ++i) {
            const u32 byte_address = address + i;
            Device* part = find_device(byte_address);
            u64 byte = part ? (part->read(byte_address, 1) & 0xFF) : 0;
            value |= byte << (8 * i);
        }
        out = value;
        note(fetch ? AccessKind::Fetch : AccessKind::Read, address, size, value, device, false);
        return true;
    }

    if (MemRegion* region = region_at(address, size)) {
        stats.ram++;
        const u8* source = region->bytes() + (address - region->base);
        u64 value = 0;
        for (unsigned i = 0; i < size; ++i) value |= static_cast<u64>(source[i]) << (8 * i);
        out = value;
        if (trace.trace_ram()) note(fetch ? AccessKind::Fetch : AccessKind::Read, address, size, value, nullptr, false);
        return true;
    }

    // Unmapped: read the pieces that do exist, and record the miss once.
    stats.unmapped++;
    last_unmapped = true;
    ZLB_LOG_DBG("bus", "unmapped read %u byte(s) at 0x%08X (pc=0x%08X)", size, address, context.pc);
    u64 value = 0;
    for (unsigned i = 0; i < size; ++i) {
        u64 byte = 0;
        Device* part = find_device(address + i);
        if (part) {
            byte = part->read(address + i, 1) & 0xFF;
        } else if (MemRegion* region = region_at(address + i, 1)) {
            byte = region->bytes()[address + i - region->base];
        } else {
            byte = unmapped_reads_zero ? 0x00 : 0xFF;
        }
        value |= byte << (8 * i);
    }
    out = value;
    note(fetch ? AccessKind::Fetch : AccessKind::Read, address, size, value, nullptr, true);
    return false;
}

bool Bus::slow_write(u32 address, unsigned size, u64 value, Device*& device) {
    device = find_device(address);
    if (device && device->handles(address + size - 1)) {
        stats.mmio++;
        device->write(address, size, value);
        note(AccessKind::Write, address, size, value, device, false);
        return true;
    }
    if (device) {
        stats.mmio++;
        for (unsigned i = 0; i < size; ++i) {
            const u32 byte_address = address + i;
            Device* part = find_device(byte_address);
            if (part) part->write(byte_address, 1, (value >> (8 * i)) & 0xFF);
        }
        note(AccessKind::Write, address, size, value, device, false);
        return true;
    }

    if (MemRegion* region = region_at(address, size)) {
        stats.ram++;
        if (!region->readonly) {
            u8* target = region->bytes() + (address - region->base);
            for (unsigned i = 0; i < size; ++i) target[i] = static_cast<u8>((value >> (8 * i)) & 0xFF);
        }
        if (trace.trace_ram()) note(AccessKind::Write, address, size, value, nullptr, false);
        return true;
    }

    stats.unmapped++;
    last_unmapped = true;
    ZLB_LOG_DBG("bus", "unmapped write %u byte(s) at 0x%08X = 0x%llX (pc=0x%08X)", size, address,
                static_cast<unsigned long long>(value), context.pc);
    for (unsigned i = 0; i < size; ++i) {
        Device* part = find_device(address + i);
        if (part) {
            part->write(address + i, 1, (value >> (8 * i)) & 0xFF);
        } else if (MemRegion* region = region_at(address + i, 1)) {
            if (!region->readonly) region->bytes()[address + i - region->base] = static_cast<u8>((value >> (8 * i)) & 0xFF);
        }
    }
    note(AccessKind::Write, address, size, value, nullptr, true);
    return false;
}

// ---------------------------------------------------------------------------
// Typed access
// ---------------------------------------------------------------------------

u8 Bus::read8(u32 address) {
    stats.reads++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1) {
        stats.ram++;
        const u8 value = page.host[address & (kPageSize - 1)];
        if (read_trap_enabled()) note_read_trap(address, 1, value);
        fast_trace(AccessKind::Read, address, 1, value);
        return value;
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 1, out, device, false);
    if (read_trap_enabled()) note_read_trap(address, 1, out);
    return static_cast<u8>(out);
}

u16 Bus::read16(u32 address) {
    stats.reads++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 2) {
        stats.ram++;
        u16 value;
        std::memcpy(&value, page.host + (address & (kPageSize - 1)), 2);
        if (read_trap_enabled()) note_read_trap(address, 2, value);
        fast_trace(AccessKind::Read, address, 2, value);
        return value;
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 2, out, device, false);
    // Every other accessor reports the slow path - read8/32/64 and all four writes.
    // read16 was the one that did not, so a 16-bit MMIO read was invisible to
    // ZLB_RTRAP: the guest's `ldrh` of the per-core window at 0xE3320000 never
    // appeared in the trace, and it looked as if the kernel only ever wrote there.
    if (read_trap_enabled()) note_read_trap(address, 2, out);
    return static_cast<u16>(out);
}

u32 Bus::read32(u32 address) {
    stats.reads++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 4) {
        stats.ram++;
        u32 value;
        std::memcpy(&value, page.host + (address & (kPageSize - 1)), 4);
        if (read_trap_enabled()) note_read_trap(address, 4, value);
        fast_trace(AccessKind::Read, address, 4, value);
        return value;
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 4, out, device, false);
    if (read_trap_enabled()) note_read_trap(address, 4, out);
    return static_cast<u32>(out);
}

u64 Bus::read64(u32 address) {
    stats.reads++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 8) {
        stats.ram++;
        u64 value;
        std::memcpy(&value, page.host + (address & (kPageSize - 1)), 8);
        if (read_trap_enabled()) note_read_trap(address, 8, value);
        fast_trace(AccessKind::Read, address, 8, value);
        return value;
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 8, out, device, false);
    if (read_trap_enabled()) note_read_trap(address, 8, out);
    return out;
}

void Bus::write8(u32 address, u8 value) {
    stats.writes++;
    clear_exclusive(address, 1);
    note_write_trap(address, 1, value);
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1) {
        stats.ram++;
        page.host[address & (kPageSize - 1)] = value;
        fast_trace(AccessKind::Write, address, 1, value);
        return;
    }
    Device* device = nullptr;
    slow_write(address, 1, value, device);
}

void Bus::write16(u32 address, u16 value) {
    stats.writes++;
    clear_exclusive(address, 2);
    note_write_trap(address, 2, value);
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 2) {
        stats.ram++;
        std::memcpy(page.host + (address & (kPageSize - 1)), &value, 2);
        fast_trace(AccessKind::Write, address, 2, value);
        return;
    }
    Device* device = nullptr;
    slow_write(address, 2, value, device);
}

void Bus::write32(u32 address, u32 value) {
    stats.writes++;
    clear_exclusive(address, 4);
    note_write_trap(address, 4, value);
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 4) {
        stats.ram++;
        std::memcpy(page.host + (address & (kPageSize - 1)), &value, 4);
        fast_trace(AccessKind::Write, address, 4, value);
        return;
    }
    Device* device = nullptr;
    slow_write(address, 4, value, device);
}

void Bus::write64(u32 address, u64 value) {
    stats.writes++;
    clear_exclusive(address, 8);
    note_write_trap(address, 8, value);
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 8) {
        stats.ram++;
        std::memcpy(page.host + (address & (kPageSize - 1)), &value, 8);
        fast_trace(AccessKind::Write, address, 8, value);
        return;
    }
    Device* device = nullptr;
    slow_write(address, 8, value, device);
}

u32 Bus::fetch32(u32 address) {
    stats.fetches++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 4) {
        stats.ram++;
        u32 value;
        std::memcpy(&value, page.host + (address & (kPageSize - 1)), 4);
        return value;
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 4, out, device, true);
    return static_cast<u32>(out);
}

u16 Bus::fetch16(u32 address) {
    stats.fetches++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1 && (address & (kPageSize - 1)) <= kPageSize - 2) {
        stats.ram++;
        u16 value;
        std::memcpy(&value, page.host + (address & (kPageSize - 1)), 2);
        return value;
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 2, out, device, true);
    return static_cast<u16>(out);
}

u8 Bus::fetch8(u32 address) {
    stats.fetches++;
    Page& page = pages_[(address >> kPageShift) & (kPageCount - 1)];
    if (page.kind == 1) {
        stats.ram++;
        return page.host[address & (kPageSize - 1)];
    }
    u64 out = 0;
    Device* device = nullptr;
    slow_read(address, 1, out, device, true);
    return static_cast<u8>(out);
}

void Bus::read_bytes(u32 address, void* out, size_t length) {
    u8* target = static_cast<u8*>(out);
    for (size_t i = 0; i < length; ++i) target[i] = read8(address + static_cast<u32>(i));
}

void Bus::write_bytes(u32 address, const void* data, size_t length) {
    const u8* source = static_cast<const u8*>(data);
    for (size_t i = 0; i < length; ++i) write8(address + static_cast<u32>(i), source[i]);
}

void Bus::memset_bytes(u32 address, u8 value, size_t length) {
    for (size_t i = 0; i < length; ++i) write8(address + static_cast<u32>(i), value);
}

bool Bus::load(u32 address, const void* data, size_t length, const std::string& tag) {
    if (length == 0) return true;
    for (size_t i = 0; i < length; ++i) note_write_trap(address + static_cast<u32>(i), 0, 0);
    if (!region_at(address, length)) ensure_ram(address, length, tag);

    // Fast path: single RAM region.
    if (MemRegion* region = region_at(address, length)) {
        std::memcpy(region->bytes() + (address - region->base), data, length);
        return true;
    }
    write_bytes(address, data, length);
    return true;
}

std::string Bus::describe_map() const {
    std::string out;
    for (const auto& region : regions_) {
        out += format("RAM  %-28s 0x%08X - 0x%08X  %-10s %s\n", region.name.c_str(), region.base,
                      region.base + region.size - 1, human_size(region.size).c_str(), region.note.c_str());
    }
    for (const auto& device : devices_) {
        out += format("DEV  %-28s 0x%08X - 0x%08X  %-10s %s\n", device->name().c_str(), device->base(),
                      device->end() - 1, human_size(device->size()).c_str(), device->summary().c_str());
    }
    return out;
}

// ---------------------------------------------------------------------------
// Save states
// ---------------------------------------------------------------------------

void Bus::save_state(StateWriter& writer) const {
    writer.begin("regions");
    writer.put_u32(static_cast<u32>(regions_.size()));
    for (const MemRegion& region : regions_) {
        writer.begin("region");
        writer.str(region.name);
        writer.put_u32(region.base);
        writer.put_u32(region.size);
        writer.put_bool(region.mapped);
        writer.put_bool(region.readonly);
        // A region with its own storage is written out through `bytes()`, so the
        // effective content is captured whether it currently serves from its own
        // vector or from an external buffer the boot chain pointed it at (the
        // CMeP mirrors its 0x40000 window onto the private SRAM while it runs).
        // A pure alias (`add_ram_alias`, empty storage) only records its
        // geometry: the machine section writes the shared buffer once.
        const bool owns = !region.data.empty();
        writer.put_bool(owns);
        if (owns) state_write_pages(writer, region.bytes(), region.data.size());
        writer.end();
    }
    writer.end();

    writer.begin("devices");
    writer.put_u32(static_cast<u32>(devices_.size()));
    for (const auto& device : devices_) {
        writer.begin("device");
        writer.str(device->name());
        // A mirror's state lives in its target, which is serialised where it is
        // registered; writing it twice would also apply it twice on load.
        if (dynamic_cast<DeviceMirror*>(device.get()) == nullptr) device->save_state(writer);
        writer.end();
    }
    writer.end();

    writer.begin("monitor");
    writer.put_u32(static_cast<u32>(kMaxReservations));
    for (const ExclusiveReservation& reservation : reservations_) {
        writer.put_bool(reservation.valid);
        writer.put_u32(reservation.address);
        writer.put_u32(reservation.size);
        writer.put_i32(reservation.owner);
        writer.put_u32(reservation.asid);
    }
    writer.put_u64(stats.reads);
    writer.put_u64(stats.writes);
    writer.put_u64(stats.fetches);
    writer.put_u64(stats.mmio);
    writer.put_u64(stats.ram);
    writer.put_u64(stats.unmapped);
    writer.put_bool(trace.trace_ram());
    writer.put_u64(trace.total());
    writer.end();
}

void Bus::load_state(StateReader& reader) {
    reader.begin("regions");
    const u32 region_count = reader.get_u32();
    if (!reader.ok()) return;
    if (region_count != regions_.size()) {
        reader.fail(format("state file: bus has %u RAM regions, this build has %zu", region_count,
                           regions_.size()));
        return;
    }
    for (MemRegion& region : regions_) {
        reader.begin("region");
        const std::string name = reader.str();
        const u32 base = reader.get_u32();
        const u32 size = reader.get_u32();
        const bool mapped = reader.get_bool();
        const bool readonly = reader.get_bool();
        const bool owns = reader.get_bool();
        if (!reader.ok()) return;
        if (name != region.name || base != region.base || size != region.size) {
            reader.fail(format("state file: RAM region '%s' does not match this build's '%s'", name.c_str(),
                               region.name.c_str()));
            return;
        }
        region.mapped = mapped;
        region.readonly = readonly;
        if (owns) {
            if (region.data.empty()) {
                reader.fail(format("state file: region '%s' carries bytes this build has no storage for",
                                   name.c_str()));
                return;
            }
            // Restore through `bytes()`: when the running machine has the region
            // pointed at an external buffer, the bytes belong there; otherwise
            // they belong in the region's own vector. Both describe the same
            // window, so the state stays self-contained either way.
            state_read_pages(reader, region.bytes(), region.data.size());
            if (!reader.ok()) return;
        }
        reader.end();
    }
    reader.end();

    reader.begin("devices");
    const u32 device_count = reader.get_u32();
    if (!reader.ok()) return;
    if (device_count != devices_.size()) {
        reader.fail(format("state file: bus has %u devices, this build has %zu", device_count,
                           devices_.size()));
        return;
    }
    for (const auto& device : devices_) {
        reader.begin("device");
        const std::string name = reader.str();
        if (!reader.ok()) return;
        if (name != device->name()) {
            reader.fail(format("state file: device '%s' does not match this build's '%s'", name.c_str(),
                               device->name().c_str()));
            return;
        }
        if (dynamic_cast<DeviceMirror*>(device.get()) == nullptr) device->load_state(reader);
        if (!reader.ok()) return;
        reader.end();
    }
    reader.end();

    reader.begin("monitor");
    const u32 reservation_count = reader.get_u32();
    if (!reader.ok()) return;
    if (reservation_count != kMaxReservations) {
        reader.fail("state file: exclusive monitor layout changed");
        return;
    }
    for (ExclusiveReservation& reservation : reservations_) {
        reservation.valid = reader.get_bool();
        reservation.address = reader.get_u32();
        reservation.size = reader.get_u32();
        reservation.owner = reader.get_i32();
        reservation.asid = reader.get_u32();
    }
    stats.reads = reader.get_u64();
    stats.writes = reader.get_u64();
    stats.fetches = reader.get_u64();
    stats.mmio = reader.get_u64();
    stats.ram = reader.get_u64();
    stats.unmapped = reader.get_u64();
    const bool trace_ram = reader.get_bool();
    const u64 trace_total = reader.get_u64();
    reader.end();
    if (!reader.ok()) return;
    trace.clear();
    trace.set_trace_ram(trace_ram);
    trace.set_total(trace_total);

    rebuild_map();
}

}  // namespace zlb
