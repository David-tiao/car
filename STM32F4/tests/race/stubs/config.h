/* Route tests always exercise the route, independent of bench-test settings. */
#include "race_config.h"
#undef RACE_PICK_DIRECT_KEY_TEST
#undef RACE_PICK_ENABLE
#define RACE_PICK_DIRECT_KEY_TEST 0U
#ifdef TEST_TASK_MISSION
#define RACE_PICK_ENABLE 1U
#undef RACE_STACK_EXTRA_STEPS
#define RACE_STACK_EXTRA_STEPS 100
#undef RACE_TAUGHT_ROUTES
/* Synthetic odometry targets, only in this native test. */
#define RACE_TAUGHT_ROUTES {1,1,0,0.1f,0,0}, {1,2,0,0.2f,0,0}, {1,3,0,0.3f,0,0}, \
    {1,4,0,0.1f,0,0}, {1,5,1,0,0.1f,0}, {1,5,2,0,0.2f,0}, {1,5,3,0,0.3f,0}, \
    {1,6,1,0,0.1f,0}, {1,6,2,0,0.2f,0}, {1,6,3,0,0.3f,0}
#else
#define RACE_PICK_ENABLE 0U
#endif
