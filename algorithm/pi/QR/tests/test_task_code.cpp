#include "pi_qr/task_code.hpp"

#include <cassert>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() {
    using namespace pi_qr;

    const auto good = parse_task_code("156+123+516+231");
    assert(good.has_value());
    assert(good->text == "156+123+516+231");
    assert(good->groups[0] == "156");
    assert(good->groups[3] == "231");
    assert(parse_task_code("  156+123+516+231\n")->text == good->text);

    const std::vector<std::string> invalid = {
        "", "156+123+516", "156+123+516+2310", "156-123-516-231",
        "116+123+516+231",  // duplicate material color
        "156+124+516+231",  // ring out of range
        "156+112+516+231",  // repeated ring
        "156+123+416+231",  // batches use different color sets
        "156+123+516+233",  // repeated ring in second batch
        "056+123+506+231",  // invalid color 0
        "156+123+516+23A",  // non-digit
        "156+123+516+231\nEXEC", // don't accept embedded text
        "156+123+516+231\0", // explicit literal backslash-zero? size differs
    };
    for (const auto& s : invalid) assert(!parse_task_code(s).has_value());

    ThreeFrameFilter filter;
    assert(!filter.observe({good->text}));
    assert(filter.streak() == 1);
    assert(!filter.observe({good->text}));
    assert(filter.streak() == 2);
    auto event = filter.observe({good->text});
    assert(event && event->text == good->text);
    assert(filter.confirmed() && filter.confirmed()->text == good->text);
    assert(!filter.observe({good->text}));  // no duplicate event
    assert(!filter.observe({}));           // disappearance breaks streak
    assert(filter.streak() == 0);
    assert(filter.confirmed().has_value()); // previous display remains marked as last

    assert(!filter.observe({good->text, good->text})); // ambiguous: 2 QRs
    assert(!filter.observe({"INVALID"}));
    assert(filter.streak() == 0);
    assert(!filter.observe({"156+123+156+321"}));
    assert(!filter.observe({"156+123+156+321"}));
    event = filter.observe({"156+123+156+321"});
    assert(event && event->text == "156+123+156+321");
    filter.clear();
    assert(!filter.confirmed());
    assert(filter.streak() == 0);

    bool caught = false;
    try { ThreeFrameFilter bad(0); }
    catch (const std::invalid_argument&) { caught = true; }
    assert(caught);

    std::cout << "task-code and consecutive-frame tests passed\n";
    return 0;
}
