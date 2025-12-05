import time                  # Standard Python time utilities (sleep, timestamps)
import socket                # For UDP networking to the Arduinos

import rclpy                 # ROS 2 Python client library
from rclpy.node import Node  # Base class for making ROS 2 nodes

# Your custom message/service types from robot_msgs package
from robot_msgs.msg import MotorArrayCmd, MotorArrayState
from robot_msgs.srv import SpinSequence


class RobotBridge(Node):                    # Define a ROS 2 node class named "RobotBridge"
    def __init__(self):
        super().__init__('robot_bridge')    # Initialize the Node with name "robot_bridge"

        # -------- Parameters (read from YAML or use these defaults) --------
        self.declare_parameter('mcu_a.ip', '192.168.1.120')   # IP for Arduino A
        self.declare_parameter('mcu_a.port', 5555)            # UDP port for Arduino A
        self.declare_parameter('mcu_b.ip', '192.168.1.121')   # IP for Arduino B
        self.declare_parameter('mcu_b.port', 5556)            # UDP port for Arduino B
        self.declare_parameter('watchdog_ms', 200)            # Safety timeout (ms) with no commands

        # Fetch parameter values (string/int) from the node's parameter server
        ip_a = self.get_parameter('mcu_a.ip').get_parameter_value().string_value
        port_a = self.get_parameter('mcu_a.port').get_parameter_value().integer_value
        ip_b = self.get_parameter('mcu_b.ip').get_parameter_value().string_value
        port_b = self.get_parameter('mcu_b.port').get_parameter_value().integer_value
        self.watchdog_ms = self.get_parameter('watchdog_ms').get_parameter_value().integer_value

        # Save addresses as (IP, port) tuples for convenience
        self.addr_a = (ip_a, port_a)
        self.addr_b = (ip_b, port_b)

        # Create one UDP socket used to send packets to both Arduinos
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Track the last time each Arduino received a command (for watchdog)
        now = time.time()
        self.last_cmd_time = {'a': now, 'b': now}

        # -------- Publishers (ROS topics we publish) --------
        # Feedback topics (you can later fill with real data from the Arduinos)
        self.pub_a = self.create_publisher(MotorArrayState, '/mcu_a/feedback', 10)  # queue size 10
        self.pub_b = self.create_publisher(MotorArrayState, '/mcu_b/feedback', 10)

        # -------- Subscribers (ROS topics we listen to) --------
        # When a MotorArrayCmd arrives for A/B, send it over UDP to the right Arduino
        self.sub_a = self.create_subscription(
            MotorArrayCmd, '/mcu_a/cmd', self.cb_cmd_a, 10)
        self.sub_b = self.create_subscription(
            MotorArrayCmd, '/mcu_b/cmd', self.cb_cmd_b, 10)

        # -------- Service (callable action) --------
        # Expose /spin_sequence so you can trigger the “A then B” demo from the CLI
        self.srv = self.create_service(
            SpinSequence, '/spin_sequence', self.handle_spin_sequence)

        # -------- Timers (periodic tasks) --------
        self.create_timer(0.05, self.timer_watchdog)   # Every 50 ms, run the watchdog
        self.create_timer(0.1, self.timer_feedback)    # Every 100 ms, publish dummy feedback

        # Log a startup message with the configured addresses and watchdog
        self.get_logger().info(
            f"RobotBridge up. A={self.addr_a}, B={self.addr_b}, watchdog={self.watchdog_ms}ms")

    # ================== Helper functions to send UDP commands ==================

    def speeds_to_pwm(self, speeds):
        # Convert an array of normalized speeds (e.g., -1..1) to a single PWM (0..255).
        # Here we just take the largest absolute value and scale it up.
        if not speeds:
            return 0
        max_abs = max(abs(s) for s in speeds)
        pwm = int(max_abs * 255)
        pwm = max(0, min(255, pwm))  # clamp to 0..255
        return pwm

    def send_spinall(self, addr, pwm, duration_ms):
        # Build a text command understood by the Arduino firmware and send via UDP
        msg = f"SPINALL {pwm} {int(duration_ms)}\n".encode()
        self.sock.sendto(msg, addr)

    def send_stop(self, addr):
        # Send a STOP command to immediately stop all motors on that Arduino
        self.sock.sendto(b"STOP\n", addr)

    # ================== Topic callbacks (when a command message arrives) ==================

    def cb_cmd_a(self, msg: MotorArrayCmd):
        # Convert the MotorArrayCmd speeds to a PWM and send it to Arduino A
        pwm = self.speeds_to_pwm(msg.speed)
        self.send_spinall(self.addr_a, pwm, msg.duration_ms)
        self.last_cmd_time['a'] = time.time()  # update watchdog timestamp

    def cb_cmd_b(self, msg: MotorArrayCmd):
        # Same as above, but for Arduino B
        pwm = self.speeds_to_pwm(msg.speed)
        self.send_spinall(self.addr_b, pwm, msg.duration_ms)
        self.last_cmd_time['b'] = time.time()

    # ================== Service handler (run “A then B” sequence) ==================

# inside RobotBridge
    def send_spin_motor(self, addr, motor_index, pwm, duration_ms):
        self.sock.sendto(f"SPIN M{motor_index} {int(pwm)} {int(duration_ms)}\n".encode(), addr)

    def handle_spin_sequence(self, req, resp):
        pwmA, msA = int(req.speed_a or 180), int(req.ms_a or 1000)
        pwmB, msB = int(req.speed_b or 180), int(req.ms_b or 1000)

        def do_one_board(addr, pwm, ms):
            for i in range(3):                     # motors 0,1,2
                self.get_logger().info(f"Seq: {addr} M{i}")
                self.send_spin_motor(addr, i, pwm, ms)
                time.sleep(ms/1000.0 + 0.1)        # wait for that motor to finish
                self.send_stop(addr)               # ensure it’s stopped
                time.sleep(0.05)

        # A then B
        do_one_board(self.addr_a, pwmA, msA)
        do_one_board(self.addr_b, pwmB, msB)

        resp.ok = True
        resp.message = "Sequential per board complete"
        return resp


    # ================== Watchdog (stop if no recent commands) ==================

    def timer_watchdog(self):
        now = time.time()
        timeout = self.watchdog_ms / 1000.0

        # If it’s been too long since last cmd for A/B, send STOP to be safe
        if now - self.last_cmd_time['a'] > timeout:
            self.send_stop(self.addr_a)
        if now - self.last_cmd_time['b'] > timeout:
            self.send_stop(self.addr_b)

    # ================== Feedback publisher (placeholder) ==================

    def timer_feedback(self):
        # If you later parse real feedback from Arduinos, publish it here.
        # For now, publish zeros so the topics exist and tools can subscribe.
        msg_a = MotorArrayState()
        msg_a.measured_speed = [0.0, 0.0, 0.0]
        msg_a.status = [0, 0, 0]
        self.pub_a.publish(msg_a)

        msg_b = MotorArrayState()
        msg_b.measured_speed = [0.0, 0.0, 0.0]
        msg_b.status = [0, 0, 0]
        self.pub_b.publish(msg_b)


def main(args=None):
    rclpy.init(args=args)        # Initialize ROS 2 for Python
    node = RobotBridge()         # Create your node instance
    try:
        rclpy.spin(node)         # Keep the node running, processing callbacks
    finally:
        node.destroy_node()      # Clean shutdown of the node
        rclpy.shutdown()         # Shut down ROS 2 client library


if __name__ == '__main__':
    main()                       # If run as a script: start main()
