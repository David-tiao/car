#include "pi_qr/task_code.hpp"

#include <algorithm>
#include <stdexcept>

namespace pi_qr {
namespace {
bool ascii_space(char c) {
    return c == ' ' || c == '\n' || c == '\r' || c == '\t';
}

bool valid_triplet(const std::string& s, const std::string& allowed) {
    if (s.size() != 3) return false;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (allowed.find(s[i]) == std::string::npos) return false;
        for (std::size_t j = 0; j < i; ++j) {
            if (s[i] == s[j]) return false;
        }
    }
    return true;
}
}  // namespace

std::optional<TaskCode> parse_task_code(std::string_view raw) {
    while (!raw.empty() && ascii_space(raw.front())) raw.remove_prefix(1);
    while (!raw.empty() && ascii_space(raw.back())) raw.remove_suffix(1);

    // Exactly 4 groups of 3 ASCII digits, separated by 3 '+' signs.
    if (raw.size() != 15 || raw[3] != '+' || raw[7] != '+' || raw[11] != '+')
        return std::nullopt;

    TaskCode task;
    task.text = std::string(raw);
    task.groups = {std::string(raw.substr(0, 3)),
                   std::string(raw.substr(4, 3)),
                   std::string(raw.substr(8, 3)),
                   std::string(raw.substr(12, 3))};

    if (!valid_triplet(task.groups[0], "123456") ||
        !valid_triplet(task.groups[1], "123") ||
        !valid_triplet(task.groups[2], "123456") ||
        !valid_triplet(task.groups[3], "123"))
        return std::nullopt;

    // Both batches must use exactly the same three material colors.
    auto first = task.groups[0];
    auto second = task.groups[2];
    std::sort(first.begin(), first.end());
    std::sort(second.begin(), second.end());
    if (first != second) return std::nullopt;
    return task;
}

ThreeFrameFilter::ThreeFrameFilter(unsigned required_frames)
    : required_frames_(required_frames) {
    if (required_frames_ < 1 || required_frames_ > 100)
        throw std::invalid_argument("required_frames must be between 1 and 100");
}

void ThreeFrameFilter::reset_pending() {
    pending_.clear();
    streak_ = 0;
}

void ThreeFrameFilter::clear() {
    reset_pending();
    confirmed_.reset();
}

std::optional<TaskCode> ThreeFrameFilter::observe(
    const std::vector<std::string>& payloads) {
    if (payloads.size() != 1) {
        reset_pending();
        return std::nullopt;
    }

    const auto parsed = parse_task_code(payloads.front());
    if (!parsed) {
        reset_pending();
        return std::nullopt;
    }

    if (parsed->text == pending_) {
        if (streak_ < required_frames_) ++streak_;
    } else {
        pending_ = parsed->text;
        streak_ = 1;
    }

    if (streak_ >= required_frames_ &&
        (!confirmed_ || confirmed_->text != parsed->text)) {
        confirmed_ = parsed;
        return parsed;
    }
    return std::nullopt;
}

}  // namespace pi_qr
