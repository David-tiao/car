#ifndef APP_MODE_H
#define APP_MODE_H

#define APP_MODE_LEGACY       0
#define APP_MODE_SERVO        1
#define APP_MODE_STEPPER      2
#define APP_MODE_RACE         3
#define APP_MODE_PI_MOTION    4

#ifndef APP_RUN_MODE
#define APP_RUN_MODE APP_MODE_RACE
#endif

#ifndef PI_MOTION_ENABLE
#define PI_MOTION_ENABLE 0
#endif

#if APP_RUN_MODE < APP_MODE_LEGACY || APP_RUN_MODE > APP_MODE_PI_MOTION
#error Invalid APP_RUN_MODE
#endif
#if (APP_RUN_MODE == APP_MODE_PI_MOTION && PI_MOTION_ENABLE != 1) || \
    (APP_RUN_MODE != APP_MODE_PI_MOTION && PI_MOTION_ENABLE != 0)
#error Pi motion requires explicit exclusive mode and enable together
#endif

#endif
