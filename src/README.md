Two-Arduino ROS 2 Robot (Wi-Fi Motor Control)

This project shows a simple but scalable framework for controlling two Arduino UNO R4 WiFi boards, each driving three motors, from ROS 2.


one ROS 2 node (robot_bridge) that talks to both Arduinos

a clean launch setup (robot_bringup)

a simple command path: ROS 2 → Wi-Fi UDP → Arduino → motors

a demo that spins each motor in sequence: A0 → A1 → A2 → B0 → B1 → B2

You can start with this and later plug in ros2_control, joysticks, navigation, etc. without throwing it away.


Two UNO R4 WiFi boards sit on your robot and control the motors.

Each Arduino:

connects to your Wi-Fi

listens on a UDP port

accepts simple text commands like:
SPIN M0 180 1000 → spin motor 0 at PWM 180 for 1000 ms
STOP → stop all motors

The robot_bridge node:

knows each Arduino’s IP + port

sends those commands when told

(for the demo) runs a sequence: motors on Arduino 1, then motors on Arduino 2.

No ROS code runs on the Arduinos. They just speak a tiny protocol over Wi-Fi.

2. Components
Arduino side (firmware)

Each UNO R4 WiFi runs a small sketch that:

connects to your Wi-Fi

listens for UDP packets

parses:

SPIN M<index> <pwm> <ms>

STOP

drives the correct motor pins

automatically stops the motor after <ms>

Board A and Board B use the same sketch, with different UDP ports.

ROS 2 side

You add three packages to your existing ROS 2 workspace:

robot_msgs
Message and service definitions:

MotorArrayCmd.msg

MotorArrayState.msg

SpinSequence.srv

robot_bridge
One node that:

reads configs (Arduino IPs, ports, watchdog time)

offers a /spin_sequence service

sends the correct UDP commands to each Arduino

(later) can subscribe to motor command topics and publish feedback

robot_bringup
One launch file + config:

bringup.launch.py starts robot_bridge

bridge.yaml stores Arduino IPs/ports + settings

You start everything with:

ros2 launch robot_bringup bringup.launch.py

Your PC and Arduinos on the same Wi-Fi network

Step 1 – Flash Arduino A

Open Arduino IDE.

Select Board: Arduino UNO R4 WiFi.

Select the correct Port.


Step 2 – Flash Arduino B


Check Serial Monitor → note B’s IP (e.g. 192.168.1.121).

Now you have:

Arduino A: IP like 192.168.1.120, UDP 5555

Arduino B: IP like 192.168.1.121, UDP 5556

Both are ready to receive commands.

Step 3 – Configure robot_bringup

In robot_bringup/config/bridge.yaml:

robot_bridge:
  ros__parameters:
    mcu_a:
      ip: 192.168.1.120    # set to Arduino A IP
      port: 5555
    mcu_b:
      ip: 192.168.1.121    # set to Arduino B IP
      port: 5556
    watchdog_ms: 500


Make sure the IPs match what the Arduinos printed.

Step 4 – Build the workspace

From your workspace root:

cd ~/ros2_ws        # or your actual path
colcon build --symlink-install


Then:

source /opt/ros/<your-distro>/setup.bash
source ~/ros2_ws/install/setup.bash


Do that in every terminal where you run ROS 2 commands.

Step 5 – Start the system

With both Arduinos powered and on Wi-Fi:

ros2 launch robot_bringup bringup.launch.py


You should see logs from robot_bridge saying it’s using those IPs/ports.

Step 6 – Run the demo sequence

In another sourced terminal:

ros2 service call /spin_sequence robot_msgs/srv/SpinSequence "{}"


The current implementation (as described) will:

On Arduino A:

spin motor 0, stop

spin motor 1, stop

spin motor 2, stop

On Arduino B:

spin motor 0, stop

spin motor 1, stop

spin motor 2, stop

Each motor is driven by a SPIN M<index> <pwm> <ms> command from robot_bridge, and each Arduino stops the motor itself after the time expires.

5. How it all talks (short version)

You call a ROS 2 service /spin_sequence.

robot_bridge sends UDP command strings to the Arduinos.

Arduinos:

parse the text,

drive the right motor pins,

stop after the requested duration.

No continuous streaming needed for this demo; timings are baked into each command.

You can later:

send commands via topics instead of a service,

add real feedback messages,

plug into ros2_control and higher-level controllers.