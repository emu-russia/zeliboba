// zeliboba - RegisterBlock save state.
//
// Every Kermit device derives from RegisterBlock, so the register image is
// serialised once here. The defaults/widths/names maps are construction-time
// configuration: they are identical in every build and are never written.
#include "hw/soc/soc_internal.h"

namespace zlb::kermit {

void RegisterBlock::save_state(StateWriter& writer) const {
    writer.map(values_, [&](u32 offset, u64 value) {
        writer.put_u32(offset);
        writer.put_u64(value);
    });
    writer.map(store_, [&](u32 offset, u64 value) {
        writer.put_u32(offset);
        writer.put_u64(value);
    });
}

void RegisterBlock::load_state(StateReader& reader) {
    reader.map(values_, [&](u32& offset, u64& value) {
        offset = reader.get_u32();
        value = reader.get_u64();
    });
    reader.map(store_, [&](u32& offset, u64& value) {
        offset = reader.get_u32();
        value = reader.get_u64();
    });
}

}  // namespace zlb::kermit
