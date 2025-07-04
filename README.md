# How to Test micro-ROS on Linux (Docker)

See the official tutorial: <https://micro.ros.org/docs/tutorials/core/first_application_linux/>

---

## 1. Start the Docker Container
```sh
docker run -it --net=host -v /dev:/dev --privileged ros:jazzy
```

## 2. Source the ROS 2 Installation
```sh
source /opt/ros/$ROS_DISTRO/setup.bash
```

## 3. Create a Workspace and Download micro-ROS Tools
```sh
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

## 4. Update Dependencies Using rosdep
```sh
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```

## 5. Install pip (if needed)
```sh
sudo apt-get install python3-pip -y
```

## 6. Build micro-ROS Tools and Source Them
```sh
colcon build
source install/local_setup.bash
```

## 7. Build and Start the micro-ROS Agent
```sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

## 8. (Optional) Enter the Container from Another Terminal
```sh
docker exec -it <container_name> bash
source /opt/ros/jazzy/setup.bash
cd microros_ws
source install/local_setup.bash
```

> **Note:** Replace `<container_name>` with the actual name or ID of your running container.

## 9. Test micro-ROS Topics
```sh
ros2 topic list
```

---

### Example: Configuring the Communication Port

To change the UART node for your board (e.g., for ESP32-C3 SuperMini):

1. Navigate to:
   `/modules/lib/micro_ros_zephyr_module/modules/libmicroros/microros_transports/serial`
2. Edit the file and change:
   ```c
   #define UART_NODE DT_NODELABEL(usart1)
   ```
   to, for example:
   ```c
   #define UART_NODE DT_NODELABEL(usb_serial)
   ```
   for ESP32-C3 SuperMini, as seen in:
   `/external/zephyr/boards/others/esp32c3_supermini/esp32c3_supermini.dts`

### Notes

- Make venv (i use zephyr ide extension vscode)
- Activate interpreter to venv in vscode
- pip install colcon-common-extensions
- pip install platformio
