from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='sim',
      executable='sim_node',
    ),
    Node(
      package='localization',
      executable='localization_node',
    ),
    Node(
      package='sim',
      executable='goal_node',
    ),
    Node(
      package='sim',
      executable='landmark_node',
    ),
    Node(
      package='gui',
      executable='gui_node',
    ),
    Node(
      package='pfc',
      executable='pfc_node',
    ),
    Node(
      package='planner',
      executable='planner_node',
      output='screen'
    )
  ])
