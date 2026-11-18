#!/bin/sh
set -eu
SRC="$1"
HERE="$(CDPATH= cd -- "$(dirname "$0")" && pwd)"
PATCH="$HERE/../STM32/Hardware"
INC="-I$HERE/stubs -I$PATCH -I$SRC/Hardware"
gcc -std=c99 -Wall -Wextra -Werror $INC "$HERE/test_v4_protocol.c" "$PATCH/race_vision_protocol.c" "$SRC/Hardware/race_fsm.c" -lm -o /tmp/v5protocol_test
/tmp/v5protocol_test
gcc -std=c99 -Wall -Wextra -Werror $INC "$HERE/test_uart_v4.c" "$PATCH/race_vision.c" "$PATCH/race_vision_protocol.c" "$SRC/Hardware/race_fsm.c" -lm -o /tmp/v5uart_test
/tmp/v5uart_test
gcc -std=c99 -Wall -Wextra -Werror $INC "$SRC/tests/race/test_race.c" "$SRC/Hardware/race_fsm.c" "$PATCH/race_vision_protocol.c" "$SRC/Hardware/race_task_plan.c" -lm -o /tmp/v5race_test
/tmp/v5race_test
python -m py_compile "$HERE/../OpenMV/main.py" "$HERE/../OpenMV/utils.py" "$HERE/../OpenMV/config.py"
python "$HERE/test_openmv_geometry.py"
python "$HERE/test_openmv_flow.py"
echo 'PASS portable checks; real hardware not tested'
