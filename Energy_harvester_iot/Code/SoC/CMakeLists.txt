cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(SoC)

target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE src/GPIO_extender.c)
target_sources(app PRIVATE src/I2C.c)
target_sources(app PRIVATE src/I2C_Switch.c)
target_sources(app PRIVATE src/TempSensor.c)
target_sources(app PRIVATE src/accuracyAlgorithms.c)
target_sources(app PRIVATE src/memory.c)
target_sources(app PRIVATE src/timers.c)

target_include_directories(app PRIVATE src)
