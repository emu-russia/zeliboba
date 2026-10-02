// zeliboba - machine save states.
//
// `Vita::save_state` writes the whole machine as one sectioned stream: the
// shared host buffers (which the buses only alias), each bus (RAM + devices +
// exclusive monitor), the three hardware blocks, the four ARM cores, the CMeP
// and the machine's own boot/scheduler flags. `load_state` reads the same
// sections back in the same order and refuses a file whose layout or media
// identity does not match the running build.
//
// The eMMC image is external input: the state records the card's path and size
// and verifies them, but never embeds the 3.8 GiB image.
#include "machine/vita.h"

#include <array>
#include <set>
#include <string>
#include <vector>

#include "common/log.h"
#include "common/state.h"
#include "common/util.h"
#include "cpu/cpu.h"
#include "hw/cmep.h"
#include "hw/emmc.h"
#include "hw/soc.h"
#include "hw/syscon.h"
#include "loader/keys.h"
#include "machine/bootchain.h"

namespace zlb {

namespace {

/// Every plain scalar the machine keeps outside its devices. The same list
/// drives save and load, so the two orders cannot drift apart.
#define ZLB_VITA_FIELDS(X)                 \
    X(u32, pc_hook_pc_)                    \
    X(i32, pc_hook_core_)                  \
    X(u64, wfe_wakeups_)                   \
    X(u64, barrier_unstuck_)               \
    X(u64, wfe_irq_wakeups_)               \
    X(u64, wfe_ticks_)                     \
    X(bool, tick_pending_)                 \
    X(u64, last_arm_instructions_)         \
    X(u32, all_wfe_streak_)                \
    X(u64, boot_fault_fixes_)              \
    X(u64, boot_pc_fixes_)                 \
    X(bool, nskbl_device_supplied_)        \
    X(u32, nskbl_service_calls_)           \
    X(u32, nskbl_service_fail_logs_)       \
    X(u32, nskbl_service_state_)           \
    X(u32, nskbl_poll_log_)                \
    X(u32, nskbl_err_trap_)                \
    X(u32, nskbl_service_completions_)     \
    X(u32, nskbl_async_bit_logs_)          \
    X(u32, instance_block_next_)           \
    X(u32, instance_blocks_supplied_)      \
    X(u32, instance_base_fixes_)           \
    X(u32, tree_node_next_)                \
    X(u32, tree_nodes_supplied_)           \
    X(u32, heap_lookup_fixes_)             \
    X(u32, lookup_object_next_)            \
    X(u32, heap_route_fixes_)              \
    X(u32, barrier_decrements_)            \
    X(u32, cookie_stamps_)                 \
    X(u32, trace_pc_hits_)                 \
    X(u32, fatal_stub_hits_)               \
    X(u32, fault_trace_hits_)              \
    X(u32, ctor_trace_hits_)               \
    X(u32, lock_trace_hits_)               \
    X(u32, objmgr_trace_hits_)             \
    X(u32, sceuid_trace_hits_)             \
    X(u32, tree_trace_hits_)               \
    X(u32, tree_fix_hits_)                 \
    X(u32, rangechk_trace_hits_)           \
    X(u32, carve_page_next_)               \
    X(u32, partition_blocks_per_class_)    \
    X(u32, partition_supplied_)            \
    X(u32, class_tables_supplied_)         \
    X(u32, class_table_va_)                \
    X(u32, kbl_vector_mirrors_)            \
    X(u32, trace_zero_hits_)               \
    X(u32, core_stack_biases_)             \
    X(u32, nskbl_lock_skips_)              \
    X(u32, nskbl_pool_next_)               \
    X(u32, nskbl_pool_supplies_)           \
    X(u32, nskbl_pool_fixes_)              \
    X(u32, magic_fixes_)                   \
    X(u32, nskbl_physpool_next_)           \
    X(u32, nskbl_physpool_fills_)          \
    X(u32, nskbl_heap_next_)               \
    X(u32, nskbl_heap_supplies_)           \
    X(bool, cmep_service_pending_)         \
    X(bool, secure_kernel_active_)         \
    X(bool, cmep_context_done_)            \
    X(u64, arm_wait_slices_)               \
    X(u32, secure_kernel_size_)            \
    X(bool, secure_modules_staged_)        \
    X(u32, context_auth_sm_pa_)            \
    X(u32, context_auth_sm_size_)          \
    X(u32, kprx_auth_sm_pa_)               \
    X(u32, kprx_auth_sm_size_)             \
    X(u32, prog_rvk_pa_)                   \
    X(u32, prog_rvk_size_)                 \
    X(bool, kbl_vectors_restored_)         \
    X(bool, pc_hook_stopped_)              \
    X(u32, second_loader_pa_)              \
    X(u32, second_loader_entry_)           \
    X(u32, kernel_entry_)                  \
    X(u32, kbl_entry_)                     \
    X(bool, built_)                        \
    X(bool, kernel_started_)               \
    X(bool, kernel_running_)               \
    X(bool, nskbl_seen_)

void write_boot_status(StateWriter& writer, const BootStatus& status) {
    writer.put_u32(static_cast<u32>(status.stage));
    writer.str(status.detail);
    writer.put_u64(status.steps_in_stage);
    writer.put_u64(status.cmep_status);
    writer.put_u32(status.arm_entry);
    writer.put_bool(status.arm_released);
}

void read_boot_status(StateReader& reader, BootStatus& status) {
    status.stage = static_cast<BootStage>(reader.get_u32());
    status.detail = reader.str();
    status.steps_in_stage = reader.get_u64();
    status.cmep_status = reader.get_u64();
    status.arm_entry = reader.get_u32();
    status.arm_released = reader.get_bool();
}

void write_slb2_entry(StateWriter& writer, const Slb2Entry& entry) {
    writer.str(entry.name);
    writer.put_u32(entry.offset);
    writer.put_u32(entry.size);
    writer.begin("Slb2Entry.data");
    writer.put_u32(static_cast<u32>(entry.data.size()));
    writer.bytes(entry.data.data(), entry.data.size());
    writer.end();
}

void read_slb2_entry(StateReader& reader, Slb2Entry& entry) {
    entry.name = reader.str();
    entry.offset = reader.get_u32();
    entry.size = reader.get_u32();
    reader.begin("Slb2Entry.data");
    const u32 size = reader.get_u32();
    if (!reader.ok()) return;
    entry.data.resize(size);
    reader.bytes(entry.data.data(), entry.data.size());
    reader.end();
}

}  // namespace

bool Vita::save_state(const std::string& path, std::string& error) {
    if (!built_) {
        error = "the machine is not built";
        return false;
    }
    if (!emmc_ || !emmc_->attached()) {
        error = "no eMMC image is attached";
        return false;
    }
    if (emmc_->writes() != 0 || emmc_->dirty()) {
        ZLB_LOG_WARN("machine",
                     "eMMC image has %llu writes; a later loadstate assumes the same image file",
                     static_cast<unsigned long long>(emmc_->writes()));
        emmc_->flush();
    }

    StateWriter body;
    body.begin("machine");
    body.put_u32(kStateFormatVersion);

    // ---- media identity --------------------------------------------------
    body.begin("media");
    body.str(emmc_->path());
    body.put_u64(emmc_->image_size());
    body.end();

    // ---- shared host buffers the buses only alias ------------------------
    body.begin("buffers");
    body.begin("shared_sram");
    state_write_pages(body, shared_sram_.data(), shared_sram_.size());
    body.end();
    body.begin("cmep_priv");
    state_write_pages(body, cmep_priv_.data(), cmep_priv_.size());
    body.end();
    body.begin("dram");
    state_write_pages(body, dram_.data(), dram_.size());
    body.end();
    body.end();

    // ---- buses (RAM regions, devices, exclusive monitor) -----------------
    body.begin("bus.arm");
    arm_bus_->save_state(body);
    body.end();
    body.begin("bus.cmep");
    cmep_bus_->save_state(body);
    body.end();
    body.begin("bus.syscon");
    syscon_bus_->save_state(body);
    body.end();

    // ---- hardware blocks (their non-device internals) --------------------
    body.begin("ernie");
    ernie_->save_state(body);
    body.end();
    body.begin("cmep_block");
    cmep_block_->save_state(body);
    body.end();
    body.begin("kermit");
    kermit_->save_state(body);
    body.end();
    body.begin("emmc");
    emmc_->save_state(body);
    body.end();

    // ---- CPU cores -------------------------------------------------------
    body.begin("cores");
    if (cmep_) {
        body.begin("core.cmep");
        cmep_->save_state(body);
        body.end();
    }
    for (int i = 0; i < kArmCoreCount; ++i) {
        Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
        if (core == nullptr) continue;
        body.begin("core.arm");
        core->save_state(body);
        body.end();
    }
    body.end();

    // ---- keys (the CMeP keyring is written while the boot runs) ----------
    body.begin("keys");
    body.map(keys_.keyring(), [&](u32 index, const KeyringSlot& slot) {
        body.put_u32(index);
        body.put_u32(slot.index);
        body.put_u32(slot.flags);
        body.bytes(slot.value.data(), slot.value.size());
        body.put_bool(slot.locked);
        body.put_bool(slot.present);
    });
    body.fixed(keys_.keyring_0501, [&](u8 byte) { body.put_u8(byte); });
    body.put_bool(keys_.keyring_0501_valid);
    body.end();

    // ---- machine flags, counters and diagnostic state --------------------
    body.begin("vita");
#define ZLB_WRITE_FIELD(type, name) body.put_##type(name);
    ZLB_VITA_FIELDS(ZLB_WRITE_FIELD)
#undef ZLB_WRITE_FIELD
    body.put_u32(static_cast<u32>(pc_hook_arch_));
    body.fixed(barrier_old_, [&](u16 value) { body.put_u16(value); });
    body.fixed(last_arm_pc_, [&](u32 value) { body.put_u32(value); });
    body.fixed(trace_ring_dumps_, [&](u32 value) { body.put_u32(value); });
    body.fixed(partition_block_next_, [&](u32 value) { body.put_u32(value); });
    body.put_i32(budget_.arm);
    body.put_i32(budget_.cmep);
    body.put_i32(budget_.rl78);
    body.begin("nskbl_service_seen");
    body.put_u32(static_cast<u32>(nskbl_service_seen_.size()));
    for (u64 value : nskbl_service_seen_) body.put_u64(value);
    body.end();
    body.begin("kbl_vectors");
    body.put_u32(static_cast<u32>(kbl_vectors_.size()));
    body.bytes(kbl_vectors_.data(), kbl_vectors_.size());
    body.end();
    body.begin("arm_cov");
    body.put_u32(static_cast<u32>(arm_cov_bits_.size()));
    body.bytes(arm_cov_bits_.data(), arm_cov_bits_.size());
    body.end();
    body.begin("mep_cov");
    body.put_u32(static_cast<u32>(mep_cov_bits_.size()));
    body.bytes(mep_cov_bits_.data(), mep_cov_bits_.size());
    body.end();
    body.begin("secure_module_sources");
    body.put_u32(static_cast<u32>(secure_module_sources_.size()));
    for (const Slb2Entry& entry : secure_module_sources_) write_slb2_entry(body, entry);
    body.end();
    body.begin("boot");
    write_boot_status(body, boot_);
    body.end();
    body.begin("milestones");
    body.put_u32(static_cast<u32>(milestones_.size()));
    for (const std::string& text : milestones_) body.str(text);
    body.end();
    body.begin("events");
    body.put_u32(static_cast<u32>(events_.size()));
    for (const std::string& text : events_) body.str(text);
    body.end();
    body.end();  // vita

    body.end();  // machine
    if (!state_write_file(path, body, error)) return false;
    ZLB_LOG_INFO("machine", "save state written: %s (%s)", path.c_str(),
                 human_size(body.size() + 20).c_str());
    return true;
}

bool Vita::load_state(const std::string& path, std::string& error) {
    if (!built_) {
        error = "the machine is not built";
        return false;
    }
    if (!emmc_ || !emmc_->attached()) {
        error = "no eMMC image is attached";
        return false;
    }
    std::vector<u8> file;
    if (!state_read_file(path, file, error)) return false;

    StateReader reader(file);
    if (!reader.begin("machine")) {
        error = reader.error();
        return false;
    }
    const u32 version = reader.get_u32();
    if (!reader.ok() || version != kStateFormatVersion) {
        error = format("state format version %u, this build writes %u", version, kStateFormatVersion);
        return false;
    }

    reader.begin("media");
    const std::string media_path = reader.str();
    const u64 media_size = reader.get_u64();
    reader.end();
    if (!reader.ok()) {
        error = reader.error();
        return false;
    }
    if (media_size != emmc_->image_size()) {
        error = format("the state was taken with a %s eMMC image, this one is %s",
                       human_size(media_size).c_str(), human_size(emmc_->image_size()).c_str());
        return false;
    }
    if (media_path != emmc_->path()) {
        ZLB_LOG_WARN("machine",
                     "loading a state taken with eMMC image '%s' onto '%s' (same size, assuming the "
                     "same content)",
                     media_path.c_str(), emmc_->path().c_str());
    }

    reader.begin("buffers");
    auto load_buffer = [&](const char* name, std::vector<u8>& bytes) {
        reader.begin(name);
        state_read_pages(reader, bytes.data(), bytes.size());
        reader.end();
    };
    load_buffer("shared_sram", shared_sram_);
    load_buffer("cmep_priv", cmep_priv_);
    load_buffer("dram", dram_);
    reader.end();
    if (!reader.ok()) {
        error = reader.error();
        return false;
    }

    reader.begin("bus.arm");
    arm_bus_->load_state(reader);
    reader.end();
    reader.begin("bus.cmep");
    cmep_bus_->load_state(reader);
    reader.end();
    reader.begin("bus.syscon");
    syscon_bus_->load_state(reader);
    reader.end();
    if (!reader.ok()) {
        error = reader.error();
        return false;
    }

    reader.begin("ernie");
    ernie_->load_state(reader);
    reader.end();
    reader.begin("cmep_block");
    cmep_block_->load_state(reader);
    reader.end();
    reader.begin("kermit");
    kermit_->load_state(reader);
    reader.end();
    reader.begin("emmc");
    emmc_->load_state(reader);
    reader.end();
    if (!reader.ok()) {
        error = reader.error();
        return false;
    }

    reader.begin("cores");
    if (cmep_) {
        reader.begin("core.cmep");
        cmep_->load_state(reader);
        reader.end();
    }
    for (int i = 0; i < kArmCoreCount; ++i) {
        Cpu* core = arm_cores_[static_cast<size_t>(i)].get();
        if (core == nullptr) continue;
        reader.begin("core.arm");
        core->load_state(reader);
        reader.end();
    }
    reader.end();
    if (!reader.ok()) {
        error = reader.error();
        return false;
    }

    reader.begin("keys");
    reader.map(keys_.keyring(), [&](u32& index, KeyringSlot& slot) {
        index = reader.get_u32();
        slot.index = reader.get_u32();
        slot.flags = reader.get_u32();
        reader.bytes(slot.value.data(), slot.value.size());
        slot.locked = reader.get_bool();
        slot.present = reader.get_bool();
    });
    reader.fixed(keys_.keyring_0501, [&](u8& byte) { byte = reader.get_u8(); });
    keys_.keyring_0501_valid = reader.get_bool();
    reader.end();
    if (!reader.ok()) {
        error = reader.error();
        return false;
    }

    reader.begin("vita");
#define ZLB_READ_FIELD(type, name) name = reader.get_##type();
    ZLB_VITA_FIELDS(ZLB_READ_FIELD)
#undef ZLB_READ_FIELD
    pc_hook_arch_ = static_cast<Arch>(reader.get_u32());
    reader.fixed(barrier_old_, [&](u16& value) { value = reader.get_u16(); });
    reader.fixed(last_arm_pc_, [&](u32& value) { value = reader.get_u32(); });
    reader.fixed(trace_ring_dumps_, [&](u32& value) { value = reader.get_u32(); });
    reader.fixed(partition_block_next_, [&](u32& value) { value = reader.get_u32(); });
    budget_.arm = reader.get_i32();
    budget_.cmep = reader.get_i32();
    budget_.rl78 = reader.get_i32();
    reader.begin("nskbl_service_seen");
    {
        const u32 count = reader.get_u32();
        nskbl_service_seen_.clear();
        for (u32 i = 0; i < count && reader.ok(); ++i) nskbl_service_seen_.insert(reader.get_u64());
    }
    reader.end();
    reader.begin("kbl_vectors");
    {
        const u32 size = reader.get_u32();
        kbl_vectors_.resize(size);
        reader.bytes(kbl_vectors_.data(), kbl_vectors_.size());
    }
    reader.end();
    reader.begin("arm_cov");
    {
        const u32 size = reader.get_u32();
        arm_cov_bits_.resize(size);
        reader.bytes(arm_cov_bits_.data(), arm_cov_bits_.size());
    }
    reader.end();
    reader.begin("mep_cov");
    {
        const u32 size = reader.get_u32();
        mep_cov_bits_.resize(size);
        reader.bytes(mep_cov_bits_.data(), mep_cov_bits_.size());
    }
    reader.end();
    reader.begin("secure_module_sources");
    {
        const u32 count = reader.get_u32();
        for (u32 i = 0; i < count && i < secure_module_sources_.size() && reader.ok(); ++i) {
            read_slb2_entry(reader, secure_module_sources_[i]);
        }
    }
    reader.end();
    reader.begin("boot");
    read_boot_status(reader, boot_);
    reader.end();
    reader.begin("milestones");
    {
        const u32 count = reader.get_u32();
        milestones_.clear();
        for (u32 i = 0; i < count && reader.ok(); ++i) milestones_.push_back(reader.str());
    }
    reader.end();
    reader.begin("events");
    {
        const u32 count = reader.get_u32();
        events_.clear();
        for (u32 i = 0; i < count && reader.ok(); ++i) events_.push_back(reader.str());
    }
    reader.end();
    reader.end();  // vita

    if (!reader.ok()) {
        error = reader.error();
        return false;
    }
    ZLB_LOG_INFO("machine", "save state loaded: %s (stage %s)", path.c_str(), to_string(boot_.stage));
    return true;
}

}  // namespace zlb
