"""Wheel speed PI control.
    ros2 launch me130_wheel wheel_pi.launch.py 
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    deadband = LaunchConfiguration("deadband")
    motor_sign = LaunchConfiguration("motor_sign")

    return LaunchDescription([
        DeclareLaunchArgument("deadband", default_value="0.00",  # TODO: <--- Change to your deadband value
                              description="from lab 1; 0.0 means no compensation"),
        DeclareLaunchArgument("motor_sign", default_value="1.0"),  # TODO: <--- Change this if your motor is spinning in the wrong direction

        DeclareLaunchArgument("target_rad_s", default_value="6.28",
                              description="wheel (output shaft) speed setpoint"),
        DeclareLaunchArgument("max_duty", default_value="1.0",
                              description="command limit in [0, 1]; write it with a "
                                          "decimal point, e.g. 0.5"),

        # Speed filter applied before the PI controller sees the speed.
        # Bigger = smoother but more lag (more overshoot at the same kp).
        DeclareLaunchArgument("speed_window_s", default_value="0.05",
                              description="window the counts are differenced over, s"),
        DeclareLaunchArgument("filter_tau_s", default_value="0.02",
                              description="first-order low-pass after the window, s; "
                                          "0.0 turns it off"),

        DeclareLaunchArgument("log", default_value="true",
                              description="record wheel_<timestamp>.csv; "
                                          "plot with: python3 src/me130_wheel/scripts/plot_wheel.py"),
        DeclareLaunchArgument("output_dir", default_value="."),

        Node(package="me130_pendulum", executable="encoder_node", name="encoder_node",
             output="screen",
             on_exit=Shutdown()),
        Node(package="me130_pendulum", executable="motor_node", name="motor_node",
             output="screen",
             parameters=[{"deadband": deadband, "motor_sign": motor_sign,
                          "max_duty": LaunchConfiguration("max_duty")}],
             on_exit=Shutdown()),

        Node(package="me130_wheel", executable="wheel_speed_pi.py",
             name="wheel_speed_node", output="screen",
             parameters=[{"target_rad_s": LaunchConfiguration("target_rad_s"),
                          # Same limit here so anti-windup knows when u is saturated.
                          "max_duty": LaunchConfiguration("max_duty"),
                          "speed_window_s": LaunchConfiguration("speed_window_s"),
                          "filter_tau_s": LaunchConfiguration("filter_tau_s"),
                          # The controller logs itself: logger_node records
                          # neither the wheel speed nor the target.
                          "log": LaunchConfiguration("log"),
                          "output_dir": LaunchConfiguration("output_dir")}],
             on_exit=Shutdown()),
    ])
