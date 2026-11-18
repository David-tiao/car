# car_interfaces

C++ ROS 2 nodes consume these `.msg` definitions. ROSIDL generates C++ types;
`.msg` syntax is the ROS 2 interface definition language and is not Python.

Grounded in David-tiao/car main (2026-11-18):
- STM32F4/Control/mecanum_chassis.h
- STM32F4/Control/Motor.h
- STM32F4/Hardware/bsp_ulog.c and bsp_ulog.h
- STM32F4/App/app_runtime.c
- STM32F4/App/race_fsm.h and race_task_plan.h
- algorithm/openmv/{main.py,serial.py,config.py,tools.py}

Do not equate ULog43 placeholder zeros with valid measurements.
Current independent algorithm/openmv uses `#` ASCII V5 while firmware
STM32F4/Hardware/vision.c expects V4 `@sequence,length,body*XX` (legacy
`#SEQ` only). A VisionEvent publisher requires an explicit protocol integration.
