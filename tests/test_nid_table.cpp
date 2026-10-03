// zeliboba - NID database self tests.
//
// The table is generated from vita_loader's db.yml.  These tests pin the shape of
// the data and one lookup that the boot investigation depends on: the display
// module's blocking call is ksceKernelDmaOpSync, which is what pointed at the DMA
// controller in the first place.
#include "test_framework.h"

#include "debug/nid_table.h"

#include <cstring>

using namespace zlb;

ZLB_TEST(nid_table_is_populated) {
    ZLB_EXPECT_TRUE(debug::nid_entry_count() > 8000u);
    const debug::NidEntry* entries = debug::nid_entries();
    ZLB_EXPECT_TRUE(entries != nullptr);
    // Sorted by NID, which the binary search relies on.
    bool sorted = true;
    for (std::size_t index = 1; index < debug::nid_entry_count(); ++index) {
        if (entries[index - 1].nid > entries[index].nid) sorted = false;
    }
    ZLB_EXPECT_TRUE(sorted);
}

ZLB_TEST(nid_lookup_names_the_blocking_import) {
    const auto matches = debug::nid_lookup(0x397A917Cu);
    ZLB_EXPECT_TRUE(!matches.empty());
    if (!matches.empty()) {
        ZLB_EXPECT_TRUE(std::strcmp(matches.front()->function, "ksceKernelDmaOpSync") == 0);
        ZLB_EXPECT_TRUE(std::strcmp(matches.front()->library, "SceDmacmgrForDriver") == 0);
    }
    // The call right before it submits the operation.
    const auto enqueue = debug::nid_lookup(0x543F54CFu);
    ZLB_EXPECT_TRUE(!enqueue.empty());
    if (!enqueue.empty()) {
        ZLB_EXPECT_TRUE(std::strcmp(enqueue.front()->function, "ksceKernelDmaOpEnQueue") == 0);
    }
    ZLB_EXPECT_TRUE(debug::nid_name(0x397A917Cu) == "SceDmacmgrForDriver::ksceKernelDmaOpSync");
}

ZLB_TEST(nid_lookup_rejects_unknown_values) {
    ZLB_EXPECT_TRUE(debug::nid_lookup(0xDEADBEEFu).empty());
    ZLB_EXPECT_TRUE(debug::nid_lookup(0x00000000u).empty());
    ZLB_EXPECT_TRUE(debug::nid_name(0xDEADBEEFu).empty());
}

ZLB_TEST(nid_search_finds_functions_and_ignores_empty_terms) {
    ZLB_EXPECT_TRUE(debug::nid_search("").empty());
    const auto matches = debug::nid_search("ksceKernelDmaOp");
    ZLB_EXPECT_TRUE(matches.size() >= 2u);
}
