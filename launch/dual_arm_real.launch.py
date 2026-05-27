import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# libjakaAPI.so lives in the jaka_driver source tree and is not installed into
# the ament prefix, so we resolve it here at parse time.
_JAKA_LIB_DIR = os.path.join(
    os.path.dirname(__file__),            # share/jaka_dual_arm/launch/
    '..', '..', '..', '..',              # → install/
    '..', 'src', 'jaka_ros2', 'src', 'jaka_driver', 'lib',
)
_JAKA_LIB_DIR = os.path.realpath(_JAKA_LIB_DIR)

# Fall back to the known absolute path if the relative resolution fails.
_JAKA_LIB_FALLBACK = os.path.expanduser(
    '~/ros2_ws/src/jaka_ros2/src/jaka_driver/lib'
)
_LIB_PATH = _JAKA_LIB_DIR if os.path.isdir(_JAKA_LIB_DIR) else _JAKA_LIB_FALLBACK

_current_ld = os.environ.get('LD_LIBRARY_PATH', '')
_LD_LIBRARY_PATH = f'{_LIB_PATH}:{_current_ld}' if _current_ld else _LIB_PATH


def generate_launch_description():
    pkg = get_package_share_directory('jaka_dual_arm')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'dual_arm_moveit.launch.py')
        )
    )

    jaka_left = Node(
        package='jaka_driver',
        executable='jaka_driver',
        name='jaka_driver',
        namespace='left',
        output='screen',
        parameters=[{'ip': '192.168.0.2'}],
        additional_env={'LD_LIBRARY_PATH': _LD_LIBRARY_PATH},
    )

    jaka_right = Node(
        package='jaka_driver',
        executable='jaka_driver',
        name='jaka_driver',
        namespace='right',
        output='screen',
        parameters=[{'ip': '192.168.0.1'}],
        additional_env={'LD_LIBRARY_PATH': _LD_LIBRARY_PATH},
    )

    return LaunchDescription([
        moveit_launch,
        jaka_left,
        jaka_right,
    ])
