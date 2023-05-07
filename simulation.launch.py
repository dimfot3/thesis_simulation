from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
import yaml
import numpy as np
import subprocess


def parse_arguments():
    args = {}
    with open('config.yaml', 'r') as file:
        args_yaml = yaml.safe_load(file)
    args['simulation_env'] = args_yaml['simulation_env']
    args['rviz2'] = args_yaml['rviz2']
    args['static_lidar'] = args_yaml['static_lidar']
    humans = [human_name for human_name in list(args_yaml.keys()) if human_name[:5] == 'human']
    lidars = [lidar_name for lidar_name in list(args_yaml.keys()) if lidar_name[:5] == 'lidar']
    args['humans'] = {}
    args['lidars'] = {}
    for human in humans:
        args['humans'][human] = args_yaml[human]
    for lidar in lidars:
        args['lidars'][lidar] = args_yaml[lidar]
    return args

def generate_lidars(lidars):
    for i, lidar_name in enumerate(lidars.keys()):
        lidar = lidars[lidar_name]
        subprocess.call(" ".join(["xacro", f"./gp_models/lidar/lidar_module.xacro", 
                        f"channels:={lidar['channels']} x:={lidar['x']} y:={lidar['y']} z:={lidar['z']} name:={lidar_name}",
                          f"> ./gp_models/lidar/lidar_{i + 1}.sdf"]), shell=True) 

def generate_humans(humans):
    print(humans)
    for human in humans.keys():
        subprocess.call(" ".join(["xacro", f"./human_models/actor_model.xacro", 
                        f"traj_path:={humans[human]['traj']}", f"velocity:={humans[human]['velocity']}",
                          f"> ./human_models/{human}.sdf"]), shell=True) 

def generate_world(world_name, lidars, humans):
    subprocess.call(" ".join(["xacro", f"./indoor_spaces/{world_name}/{world_name}.xacro", 
                        f"lidar_n:={len(list(lidars.keys()))}", f"human_n:={len(list(humans.keys()))}"
                          f"> ./indoor_spaces/{world_name}/{world_name}.sdf"]), shell=True) 

def generate_launch_description():
    # load arguments
    args = parse_arguments()
    simulation_env_arg = args['simulation_env']
    rviz2_arg = args['rviz2']
    generate_lidars(args['lidars'])
    generate_humans(args['humans'])
    generate_world(args['simulation_env'], args['lidars'], args['humans'])
    # set environment paths
    os.environ['AMENT_PREFIX_PATH'] +=f':{os.getcwd()}/ros_packages/install/gazebo_to_ros2'
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] =f':{os.getcwd()}/ros_packages/build/gazebo_to_ros2'
    lidar_names = [*args['lidars'].keys()]
    lidar_args = np.array([])
    for lidar in args['lidars'].keys():
        lidar_args = np.append(lidar_args, (f'{lidar} {args["lidars"][lidar]["x"]} {args["lidars"][lidar]["y"]} {args["lidars"][lidar]["z"]} ' + \
        f'{args["lidars"][lidar]["rx"]} {args["lidars"][lidar]["ry"]} {args["lidars"][lidar]["rz"]}').split(" "))
    print(lidar_args)
    # init lidar tf static publisher
    lidar_tf_node = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_to_ros2', 'lidar_tf_publisher', *lidar_args],
        output='screen'
    )

    # init lidar gazebo to ros2 node
    lidar_node = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_to_ros2', 'lidar_republisher', *lidar_names],
        output='screen'
    )

    # init simulation node
    gazebo_node = ExecuteProcess(
    cmd=['gz', 'sim', '-v3',f'indoor_spaces/{simulation_env_arg}/{simulation_env_arg}.sdf'],
    output='screen')

    # init rviz2 monitor
    rviz2_node = ExecuteProcess(
    cmd=['rviz2'],
    output='screen')

    # init rviz2 monitor
    ld_pos_node = ExecuteProcess(
    cmd=['python3', 'lidar_positioning.py'],
    output='screen')

    # create and return launcher
    ld = LaunchDescription()
    ld.add_action(lidar_node)
    if(args['static_lidar']):
        ld.add_action(lidar_tf_node)
    else:
        ld.add_action(ld_pos_node)
    ld.add_action(gazebo_node)
    if rviz2_arg: ld.add_action(rviz2_node)
    return ld


