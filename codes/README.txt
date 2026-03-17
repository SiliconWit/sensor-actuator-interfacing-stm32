Sensor-Actuator Interfacing STM32: Reference Projects

Each folder contains reference files for the corresponding lesson.

lesson-01-proximity-alarm/
  ProximityAlarm.ioc    CubeMX config for Lesson 1

lesson-10-capstone/
  MultiSensorLogger.ioc CubeMX config for capstone (all peripherals)
  main.c                Step 1 working code: internal temp, OLED,
                        potentiometer threshold, alarm GPIO, buttons

To use the .ioc files:
  1. Open in STM32CubeMX (File > Open Existing Project)
  2. Click GENERATE CODE
  3. Import into STM32CubeIDE (File > Import > General > Existing Projects)

To use main.c:
  Copy main.c into your project's Core/Src/ folder, replacing the
  CubeMX-generated main.c. The USER CODE sections contain all application
  logic. Rebuild in CubeIDE.
