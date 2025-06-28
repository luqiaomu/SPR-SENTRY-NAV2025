import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    # 顶层命名空间 launch 参数
    bringup_dir = get_package_share_directory("rm_decision_cpp")

    namespace = LaunchConfiguration("namespace")

    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # 声明命名空间参数
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="Top-level namespace",
    )

    # 声明参数文件路径参数
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "node_params01.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    param_substitutions = {}  # 如果你不需要动态替换 use_sim_time 等参数

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,  # <== 这里必须传入，即使为空
            convert_types=True,
        ),
        allow_substs=True,
    )


    # 启动组
    bringup_cmd_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),  # 使用命名空间
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),

            Node(
                package='rm_decision_cpp',
                executable='tree_exec_node',
                name='tree_exec',
                parameters=[configured_params],  # <- 使用处理后的参数文件
                output='screen',
                # namespace=namespace,  # 可写可不写
            )
        ]
    )

    # 添加所有 launch 动作
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(bringup_cmd_group)

    return ld






















# # Copyright (c) 2018 Intel Corporation
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# """
# Example for spawning multiple robots in Gazebo.

# This is an example on how to create a launch file for spawning multiple robots into Gazebo
# and launch multiple instances of the navigation stack, each controlling one robot.
# The robots co-exist on a shared environment and are controlled by independent nav stacks.
# """

# import os.path

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
#                             IncludeLaunchDescription, LogInfo)
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch_ros.actions import Node,LoadComposableNodes
# from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     config = os.path.join(
#         get_package_share_directory('rm_decision_cpp'), 'config', 'node_params.yaml')


#     # Create the launch description and populate
#     ld = LaunchDescription()
#     demo_cmd = Node(
#         package='rm_decision_cpp',
#         name='tree_exec',
#         executable='tree_exec_node',
#         parameters=[config],
#         arguments=['--ros-args', '--log-level', 'info'],
#         output='screen')
#     ld.add_action(demo_cmd)

#     return ld