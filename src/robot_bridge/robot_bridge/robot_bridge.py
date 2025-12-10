#!/usr/bin/env python3
import time
import socket

import rclpy
from rclpy.node import Node

from robot_msgs.msg import MotorArrayState
from robot_msgs.srv import SpinSequence   # we’ll reuse this srv name for now


class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # -------- Parameters --------
        self.declare_parameter('mcu_a.ip', '192.168.1.190')  # IP for Arduino A
        self.declare_parameter('mcu_a.port', 5555)           # UDP port for Arduino A
        self.declare_parameter('mcu_b.ip', '192.168.1.121')  # IP for Arduino B
        self.declare_parameter('mcu_b.port', 5556)           # UDP port for Arduino B
        self.declare_parameter('watchdog_ms', 200)           # Safety timeout (ms) with no commands

        ip_a = self.get_parameter('mcu_a.ip').get_parameter_value().string_value
        port_a = self.get_parameter('mcu_a.port').get_parameter_value().integer_value
        ip_b = self.get_parameter('mcu_b.ip').get_parameter_value().string_value
        port_b = self.get_parameter('mcu_b.port').get_parameter_value().integer_value
        self.watchdog_ms = self.get_parameter('watchdog_ms').get_parameter_value().integer_value

        self.addr_a = (ip_a, port_a)
        self.addr_b = (ip_b, port_b)

        # One UDP socket for both MCUs
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        now = time.time()
        self.last_cmd_time = {'a': now, 'b': now}

        # -------- Publishers (feedback placeholders) --------
        self.pub_a = self.create_publisher(MotorArrayState, '/mcu_a/feedback', 10)
        self.pub_b = self.create_publisher(MotorArrayState, '/mcu_b/feedback', 10)

        # -------- (Optional) Command subscribers – currently unused for steppers --------
        # You can rewire these later when you have a MotorArrayCmd definition for stepper moves.
        # For now, they just log that they're not implemented.
        # from robot_msgs.msg import MotorArrayCmd
        # self.sub_a = self.create_subscription(MotorArrayCmd, '/mcu_a/cmd', self.cb_cmd_a, 10)
        # self.sub_b = self.create_subscription(MotorArrayCmd, '/mcu_b/cmd', self.cb_cmd_b, 10)

        # -------- Service: run sequence A then B --------
        self.srv = self.create_service(SpinSequence, '/spin_sequence', self.handle_spin_sequence)

        # -------- Timers --------
        self.create_timer(0.05, self.timer_watchdog)
        self.create_timer(0.1, self.timer_feedback)

        self.get_logger().info(
            f"RobotBridge up for CNC steppers. A={self.addr_a}, B={self.addr_b}, "
            f"watchdog={self.watchdog_ms}ms"
        )

    # ================== UDP helpers ==================

    def send_move(self, addr, axis: str, steps: int, us_per_step: int):
        """
        Send a MOVE command understood by the new CNC/stepper firmware:
          MOVE X <steps> <us_per_step>
        """
        axis = axis.upper()
        cmd = f"MOVE {axis} {int(steps)} {int(us_per_step)}\n"
        self.get_logger().info(f"UDP -> {addr}: {cmd.strip()}")
        self.sock.sendto(cmd.encode(), addr)

    def send_stop(self, addr):
        self.get_logger().debug(f"UDP -> {addr}: STOP")
        self.sock.sendto(b"STOP\n", addr)

    # ================== (Optional) Topic callbacks – currently disabled ==================
    # If/when you define a stepper-style MotorArrayCmd, you can re-enable these
    # and map its fields into MOVE commands.

    # def cb_cmd_a(self, msg: MotorArrayCmd):
    #     self.get_logger().warn("cb_cmd_a not yet implemented for CNC steppers")

    # def cb_cmd_b(self, msg: MotorArrayCmd):
    #     self.get_logger().warn("cb_cmd_b not yet implemented for CNC steppers")

    # ================== Service: sequential moves A then B ==================

    def handle_spin_sequence(self, req, resp):
        """
        We reuse the SpinSequence srv, but reinterpret its fields for steppers:
          - req.speed_a:  steps per axis on board A   (e.g. 1000)
          - req.ms_a:     microseconds per step on A  (e.g. 800)
          - req.speed_b:  steps per axis on board B   (e.g. 1000)
          - req.ms_b:     microseconds per step on B  (e.g. 800)

        For each board:
          MOVE X ...
          MOVE Y ...
          MOVE Z ...
        in sequence.
        """
        steps_a = int(req.speed_a or 1000)
        us_a    = int(req.ms_a or 800)
        steps_b = int(req.speed_b or 1000)
        us_b    = int(req.ms_b or 800)

        axes = ['X', 'Y', 'Z']

        def do_one_board(name: str, addr, steps: int, us_per_step: int):
            for axis in axes:
                self.get_logger().info(
                    f"Sequence on {name}: MOVE {axis} {steps} {us_per_step}"
                )
                self.send_move(addr, axis, steps, us_per_step)

                # Estimated duration: each step has two edges (HIGH + LOW)
                est_time = abs(steps) * us_per_step * 2 / 1_000_000.0
                time.sleep(est_time + 0.25)  # add small margin

                # Safety STOP after each axis
                self.send_stop(addr)
                time.sleep(0.05)

        # Run A then B
        do_one_board("A", self.addr_a, steps_a, us_a)
        do_one_board("B", self.addr_b, steps_b, us_b)

        resp.ok = True
        resp.message = "Stepper sequence A then B complete"
        return resp

    # ================== Watchdog ==================

    def timer_watchdog(self):
        now = time.time()
        timeout = self.watchdog_ms / 1000.0

        # Only makes sense if we start using command topics again,
        # but we'll keep it as a safety that periodically sends STOP.
        if now - self.last_cmd_time['a'] > timeout:
            self.send_stop(self.addr_a)
        if now - self.last_cmd_time['b'] > timeout:
            self.send_stop(self.addr_b)

    # ================== Feedback publisher (dummy for now) ==================

    def timer_feedback(self):
        msg_a = MotorArrayState()
        msg_a.measured_speed = [0.0, 0.0, 0.0]
        msg_a.status = [0, 0, 0]
        self.pub_a.publish(msg_a)

        msg_b = MotorArrayState()
        msg_b.measured_speed = [0.0, 0.0, 0.0]
        msg_b.status = [0, 0, 0]
        self.pub_b.publish(msg_b)


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
