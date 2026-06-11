from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Use simulation time for all nodes
        Node(
            package='modelo_omnidireccional',
            executable='omni_odometry_node',
            name='omni_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'stepping': 0.1},
                {'trajectory_type': 'square'},
                {'total_time': 50.0},
                {'amplitude': 1.0},
                {'cycles': 1.0},
                {'square_side': 2.0},
                {'square_speed': 0.2},
                {'spline_waypoints': [
                    0.,  0., 0., -2.35619,
                    10., 2., 0., -0.78539,
                    20., 2., 2.,  0.78539,
                    30., 0., 2.,  2.35619,
                    40., 0., 0., -2.35619,
                ]}
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)