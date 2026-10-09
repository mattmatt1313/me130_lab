#!/usr/bin/env python3
"""Wheel speed PI control -- the node students edit.

Reads /encoder/state, writes /motor/command. Like controller_node it knows
nothing about the driver, the deadband or the wiring; motor_node owns that.

Speed is in wheel (output shaft) rad/s. The setpoint is the target_rad_s
parameter or the latest /wheel/target_rad_s message (std_msgs/Float64). The
filtered speed is published on /wheel/speed_rad_s for rqt_plot.

With log:=true every control step is written to wheel_<timestamp>.csv in
output_dir; plot it with
    python3 ~/me130_lab/ros2_ws/src/me130_wheel/scripts/plot_wheel.py


The gains default to ZERO, so a fresh launch does not spin the wheel.
"""
import math
import os
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import Float64

from me130_interfaces.msg import EncoderState, MotorCommand


class WheelSpeedPI(Node):
    def __init__(self):
        super().__init__("wheel_speed_node")

        # Students set these. Units: duty per (rad/s), and duty per rad.
        # dynamic_typing so "target_rad_s:=10" works as well as "10.0".
        number = ParameterDescriptor(dynamic_typing=True)
        self.target_rad_s = float(self.declare_parameter("target_rad_s", 0.0, number).value)

        # 20 counts per motor rev * 25:1 gearbox (me130_pendulum/hardware.hpp).
        self.counts_per_rev = self.declare_parameter("counts_per_rev", 500.0).value
        # Speed estimate, before the student's controller sees it. The encoder
        # rate is counts differenced over one 5 ms sample, so a single count is
        # ~2.5 rad/s and the raw rate sawtooths as counts land in alternate
        # samples. Instead difference the absolute counts over a window: one
        # count over 50 ms is ~0.25 rad/s, for ~25 ms of lag. Then a light
        # first-order low-pass on top. Longer window/tau = smoother but laggier.
        self.speed_window_s = self.declare_parameter("speed_window_s", 0.05).value
        self.filter_tau_s = self.declare_parameter("filter_tau_s", 0.02).value
        self.count_history = deque()   # (stamp, counts) inside the window
        self.max_duty = self.declare_parameter("max_duty", 1.0).value
        # A wrong motor_sign turns PI into positive feedback: the output pins at
        # the limit while the wheel spins the other way. Stop if that persists.
        self.runaway_s = self.declare_parameter("runaway_timeout_s", 0.5).value
        log = self.declare_parameter("log", False).value
        output_dir = self.declare_parameter("output_dir", ".").value

        self.speed_rad_s = 0.0
        self.integral = 0.0
        self.last_u = 0.0
        self.last_stamp = None
        self.runaway_elapsed = 0.0
        self.tripped = False

        self.cmd_pub = self.create_publisher(MotorCommand, "/motor/command",
                                             qos_profile_sensor_data)
        self.speed_pub = self.create_publisher(Float64, "/wheel/speed_rad_s", 10)
        self.create_subscription(EncoderState, "/encoder/state", self.on_encoder,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, "/wheel/target_rad_s", self.on_target, 10)
        self.add_on_set_parameters_callback(self.on_params)

        self.csv = None
        if log:
            path = os.path.join(output_dir, time.strftime("wheel_%Y%m%d_%H%M%S.csv"))
            self.csv = open(path, "w")
            self.csv.write("t_s,mode,target_rad_s,speed_rad_s,u,u_unsaturated\n")
            self.t0 = self.get_clock().now()
            self.get_logger().info(f"logging to {os.path.abspath(path)}")

    # ------------------------------------------------------------------ #
    # TODO: Implement your controller below
    # ------------------------------------------------------------------ #
    def pi_control(self, speed_cmd, speed, dt):
        """Return the unsaturated PWM duty command u

        speed_cmd -- commanded wheel speed, rad/s
        speed     -- measured (filtered) wheel speed, rad/s
        dt        -- time since the last call, s
        """

        # TODO: self.integral and self.last_u are "object attributes" 
        # that are initialized to zero. Changes to these variables persist across 
        # function calls.
        # * self.integral : update this to integrate the error across multiple calls of pi_control
        # * self.last_u : in line (131) this is set to the last saturated input which is useful for the Back-Calculation anti-windup method. Don't update this variable yourself. 
        self.integral 
        
        u = 0 # TODO: implement your controller here

        return u
    # ------------------------------------------------------------------ #


    def on_encoder(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        dt = (stamp - self.last_stamp).nanoseconds * 1e-9 if self.last_stamp else 0.0
        self.last_stamp = stamp
        dt_ok = 0.0 < dt < 0.5

        raw = self.windowed_speed(stamp, msg.counts)
        if dt_ok:
            alpha = dt / (self.filter_tau_s + dt)
            self.speed_rad_s += alpha * (raw - self.speed_rad_s)
        else:
            self.speed_rad_s = raw
        self.speed_pub.publish(Float64(data=self.speed_rad_s))

        if self.tripped or not dt_ok:
            self.publish(0.0, "coast")
            return

        # The student's controller returns an unclamped command; limit it here.
        # last_u is the command actually applied, which is what an anti-windup
        # check against max_duty needs.
        u_raw = self.pi_control(self.target_rad_s, self.speed_rad_s, dt)
        u = max(-self.max_duty, min(self.max_duty, u_raw))
        self.last_u = u

        # Runaway check: pinned at the limit, and the wheel turning against it.
        against = (abs(u) >= self.max_duty and self.speed_rad_s * u < 0.0
                   and abs(self.speed_rad_s) > 3.0)
        self.runaway_elapsed = self.runaway_elapsed + dt if against else 0.0
        if self.runaway_elapsed > self.runaway_s:
            self.tripped = True
            self.publish(0.0, "coast")
            self.get_logger().error(
                "Wheel is turning against a saturated command -- stopped. "
                "motor_sign is probably wrong; restart with a flipped motor_sign:=+/-1.0.")
            return

        self.publish(u, "pi", u_raw)
        self.get_logger().info(
            f"target {self.target_rad_s:6.2f} rad/s  speed {self.speed_rad_s:6.2f} rad/s  u {u:+.3f}",
            throttle_duration_sec=1.0)

    def windowed_speed(self, stamp, counts):
        """Wheel speed (rad/s) from the change in counts across speed_window_s."""
        self.count_history.append((stamp, counts))
        # Keep the oldest sample that is still at least one window old, so the
        # difference always spans the full window once it has filled.
        while (len(self.count_history) > 2 and
               (stamp - self.count_history[1][0]).nanoseconds * 1e-9 >= self.speed_window_s):
            self.count_history.popleft()
        t_old, c_old = self.count_history[0]
        span = (stamp - t_old).nanoseconds * 1e-9
        if span <= 0.0:
            return 0.0
        return (counts - c_old) / span / self.counts_per_rev * 2.0 * math.pi

    def on_target(self, msg):
        self.target_rad_s = msg.data
        self.get_logger().info(f"target = {self.target_rad_s:.2f} rad/s")

    def on_params(self, params):
        for p in params:
            if p.name in ("target_rad_s",) and p.type_ not in (
                    Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(successful=False, reason=f"{p.name} must be a number")
            if p.name == "target_rad_s":
                self.target_rad_s = float(p.value)
                self.get_logger().info(f"target = {self.target_rad_s:.2f} rad/s")
        return SetParametersResult(successful=True)

    def publish(self, u, mode, u_raw=None):
        """Send u to motor_node. u_raw is the controller output before clamping,
        logged for comparison; it defaults to u."""
        u_raw = u if u_raw is None else u_raw
        now = self.get_clock().now()
        msg = MotorCommand()
        msg.header.stamp = now.to_msg()
        msg.u = float(u)
        msg.mode = mode
        msg.segment = 1
        self.cmd_pub.publish(msg)

        if self.csv:
            t = (now - self.t0).nanoseconds * 1e-9
            self.csv.write(f"{t:.4f},{mode},{self.target_rad_s:.4f},{self.speed_rad_s:.4f},"
                           f"{u:.4f},{u_raw:.4f}\n")


def main():
    rclpy.init()
    node = WheelSpeedPI()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, "coast")
        if node.csv:
            node.csv.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
