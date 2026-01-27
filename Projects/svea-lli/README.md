# Project Overview

This project is designed to implement a control system using the Zephyr RTOS, integrating remote control inputs, servo motor control, and sensor data processing. The system is structured to ensure efficient handling of inputs and commands while maintaining responsiveness and reliability.

## How to flash
from the repo root
`west flash -d Projects/svea-lli/build/clicker4_stm32f7 -r openocd -- \
  --config interface/cmsis-dap.cfg --config target/stm32f7x.cfg`

## How to debug and develop

Use the [Zephyr IDE extension](https://marketplace.visualstudio.com/items?itemName=mylonics.zephyr-ide)

Flash the debug build and run
`west rtt -d Projects/svea-lli/build/clicker4_stm32f7_DEBUG -r openocd`
to get logs

or, while debugging with vscode extension [Cortex-Debug](https://marketplace.visualstudio.com/) run `nc 127.0.0.1 5555` in a new terminal to get logs

Haven't had much luck devloping on windows (big surprise), linux or mac strongly recommended

## Directory Structure

- **boards/**: Contains the Device Tree overlay file for specific board configurations.
- **include/**: Header files that declare functions and data structures for various components of the system.
- **src/**: Source files that implement the functionality of the system, including control logic, input handling, and communication with the micro-ROS framework.
- **CMakeLists.txt**: CMake configuration file for building the application.
- **prj.conf**: Configuration file for Zephyr project settings.

## Components

1. **Device Tree Overlay**: 
   - Located in `boards/myboard.overlay`, this file defines the hardware configurations, including aliases for PWM inputs and outputs.

2. **Control Logic**:
   - The arbiter thread is implemented in `src/control.c`, processing inputs from remote control and ROS commands to determine servo actions.

3. **PWM Input Handling**:
   - The PWM input capture functionality is implemented in `src/rc_input.c`, which initializes input channels and handles captured signals.

4. **Remote Control Management**:
   - The state of remote control inputs is managed in `src/remote.c`, including link monitoring and message queuing.

5. **Micro-ROS Integration**:
   - The integration with the micro-ROS framework is handled in `src/ros_iface.c`, managing initialization and message processing.

6. **Sensor Data Processing**:
   - Sensor initialization and data reading are implemented in `src/sensors.c`, ensuring devices are ready before fetching data.

7. **Servo Control**:
   - Servo motor control logic is implemented in `src/servo.c`, handling requests to update servo positions based on input changes.

## Setup Instructions

1. **Clone the Repository**: 
   - Clone the project repository to your local machine.

2. **Install Dependencies**: 
   - Ensure you have the Zephyr SDK and other necessary tools installed.

3. **Build the Project**: 
   - Navigate to the project directory and run the build command using CMake.

4. **Flash the Firmware**: 
   - Connect your target board and flash the compiled firmware.

5. **Run the Application**: 
   - Start the application and monitor the output for any debug information.

## Functionality

The system is designed to handle multiple input sources, including remote control signals and commands from the micro-ROS framework. It processes these inputs to control servo motors, ensuring smooth and responsive operation. The architecture is built to be fault-tolerant, with mechanisms in place for monitoring link status and ensuring that the system can recover from potential failures.