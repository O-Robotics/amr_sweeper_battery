from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="amr_sweeper_battery",
            executable="amr_sweeper_battery_node",
            name="amr_sweeper_battery",
            parameters=[{
                "can_interface": "can0",
                "timer_period": 1.0,
                "priority": 0x18,
                "bms_address": 0x01,
                "pc_address": 0x40,
            }],
        )
    ])
