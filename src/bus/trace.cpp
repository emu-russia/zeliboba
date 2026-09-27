#include "bus/trace.h"

#include <algorithm>

namespace zlb {

const char* to_string(AccessKind kind) {
    switch (kind) {
        case AccessKind::Read: return "R";
        case AccessKind::Write: return "W";
        case AccessKind::Fetch: return "F";
    }
    return "?";
}

TraceLog::TraceLog(size_t capacity) { records_.resize(capacity == 0 ? 1 : capacity); }

void TraceLog::set_capacity(size_t capacity) {
    records_.assign(capacity == 0 ? 1 : capacity, AccessRecord{});
    head_ = 0;
    count_ = 0;
}

void TraceLog::push(const AccessRecord& record) {
    records_[head_] = record;
    head_ = (head_ + 1) % records_.size();
    if (count_ < records_.size()) ++count_;
    ++total_;
}

std::vector<AccessRecord> TraceLog::tail(size_t count) const {
    if (count > count_) count = count_;
    std::vector<AccessRecord> out;
    out.reserve(count);
    for (size_t i = count; i > 0; --i) {
        size_t index = (head_ + records_.size() - i) % records_.size();
        out.push_back(records_[index]);
    }
    return out;
}

std::vector<AccessRecord> TraceLog::all() const { return tail(count_); }

std::vector<AccessRecord> TraceLog::since(u64 sequence, size_t max_count) const {
    std::vector<AccessRecord> out;
    if (max_count == 0) return out;
    size_t scanned = 0;
    for (size_t i = 0; i < count_ && scanned < max_count; ++i) {
        const size_t index = (head_ + records_.size() - 1 - i) % records_.size();
        const AccessRecord& record = records_[index];
        if (record.sequence < sequence) break;
        out.push_back(record);
        ++scanned;
    }
    std::reverse(out.begin(), out.end());
    return out;
}

std::vector<AccessRecord> TraceLog::find(u32 address, u32 mask, size_t max_count) const {
    std::vector<AccessRecord> out;
    for (size_t i = count_; i > 0; --i) {
        size_t index = (head_ + records_.size() - i) % records_.size();
        const AccessRecord& record = records_[index];
        if ((record.address & mask) == (address & mask)) {
            out.push_back(record);
            if (out.size() >= max_count) break;
        }
    }
    return out;
}

void TraceLog::clear() {
    head_ = 0;
    count_ = 0;
}

}  // namespace zlb
