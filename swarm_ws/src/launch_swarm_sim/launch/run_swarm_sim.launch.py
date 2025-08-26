from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Set up environment variables
    ld.add_action(SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='110'
    ))
    
    px4_dir = os.path.join(os.path.expanduser('~'), 'PX4-Autopilot')
    swarm_ws_dir = os.path.join(os.path.expanduser('~'), 'swarm_ws')
    gazebo_resources = os.path.join(px4_dir, 'Tools/simulation/gz')

    # Get the full environment with ROS sourcing
    def get_ros_env():
        env = os.environ.copy()
        return {
            **env,
            'PATH': f"/opt/ros/humble/bin:{env.get('PATH', '')}",
            'PYTHONPATH': f"/opt/ros/humble/lib/python3.10/site-packages:{env.get('PYTHONPATH', '')}",
            'ROS_VERSION': '2',
            'ROS_DISTRO': 'humble'
        }

    # Function to spawn PX4 instances with environment
    def get_px4_env(instance_id, pose, standalone=True):
        env = os.environ.copy()
        env.update({
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_GZ_MODEL_POSE': pose,
            'PX4_GZ_MODEL': 'gz_x500',
            'GZ_SIM_RESOURCE_PATH': gazebo_resources,
            'GZ_MODEL_PATH': os.path.join(gazebo_resources, 'models')
        })
        if standalone:
            env['PX4_GZ_STANDALONE'] = '1'
        else:
            env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.path.join(px4_dir, 'build/px4_sitl_default/build_gz')
            env['GZ_SIM_RESOURCE_PATH'] = gazebo_resources + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')
        return env

    # Run Gazebo simulation (main instance)
    gazebo_process = ExecuteProcess(
        cmd=[os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'), '-i', '1'],
        cwd=px4_dir,
        env=get_px4_env('1', '1.0,1.0,0.1', False),  # Main instance position
        output='screen'
    )
    ld.add_action(gazebo_process)

    # Original drone positions
    drone_configs = [
        {'id': '1', 'pose': '1.0,1.0,0.1'},
        {'id': '2', 'pose': '0.0,0.0,0.1'},
        {'id': '3', 'pose': '0.0,1.0,0.1'},
        {'id': '4', 'pose': '0.0,2.0,0.1'}
    ]

    # Spawn all drones with original timing (7s, 3s, 3s, 3s)
    spawn_delays = [7.0, 3.0, 3.0, 3.0]
    for delay, drone in zip(spawn_delays, drone_configs):
        process = ExecuteProcess(
            cmd=[os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'), '-i', drone['id']],
            cwd=px4_dir,
            env=get_px4_env(drone['id'], drone['pose']),
            output='screen'
        )
        ld.add_action(TimerAction(
            period=delay,
            actions=[process]
        ))

    # Start MicroXRCEAgent after last drone spawns (9s)
    microxrce_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        additional_env={'ROS_DOMAIN_ID': '0'},
        cwd=px4_dir,
        output='screen'
    )
    ld.add_action(TimerAction(
        period=9.0,  
        actions=[microxrce_process]
    ))

    # Launch control nodes 11 seconds after MicroXRCEAgent starts (total 20s)
    takeoff_delay = 0
    for drone in drone_configs:
        control_node = ExecuteProcess(
            cmd=['ros2', 'run', 'swarm_control', 'offboard_takeoff', 
                 '--ros-args', '-p', f'drone_ID:={drone["id"]}'],
            cwd=swarm_ws_dir,
            env=get_ros_env(),
            output='screen'
        )
        ld.add_action(TimerAction(
            period=20.0 + takeoff_delay,
            actions=[control_node]
        ))
        takeoff_delay += 1

    return ld