#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pi_qr {

// The task-code grammar matches algorithm/openmv/openmv_all/tools.py.
// Original scanned text is treated as untrusted data and is NEVER executed.
struct TaskCode {
    std::string text;                       // canonical: 156+123+516+231
    std::array<std::string, 4> groups;      // colors, rings, colors, rings
};

std::optional<TaskCode> parse_task_code(std::string_view raw);

class ThreeFrameFilter {
public:
    explicit ThreeFrameFilter(unsigned required_frames = 3);

    // One camera frame -> all decoded QR payloads from that single frame.
    // Exactly one valid code must appear in consecutive frames.
    // Returns a code only when it changes to a newly confirmed value.
    std::optional<TaskCode> observe(const std::vector<std::string>& payloads);

    unsigned streak() const noexcept { return streak_; }
    unsigned required_frames() const noexcept { return required_frames_; }
    const std::optional<TaskCode>& confirmed() const noexcept { return confirmed_; }
    void clear();

private:
    void reset_pending();
    unsigned required_frames_;
    unsigned streak_ = 0;
    std::string pending_;
    std::optional<TaskCode> confirmed_;
};

}  // namespace pi_qr
