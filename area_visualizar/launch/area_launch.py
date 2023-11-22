from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch引数の宣言
    declare_eagleye_area_arg = DeclareLaunchArgument(
        'eagleye_area', default_value='default_area',
        description='Eagleye area parameter value')

    declare_switching_area_arg = DeclareLaunchArgument(
        'switching_area', default_value='default_switching_area',
        description='Switching area parameter value')

    # Areaノードの設定
    area_node = Node(
        package='your_package_name',  # あなたのパッケージ名に置き換えてください
        executable='area',  # 実行可能ファイルの名前
        name='area_node',
        output='screen',
        parameters=[{
            'eagleye_area': LaunchConfiguration('eagleye_area'),
            'switching_area': LaunchConfiguration('switching_area'),
        }],
    )

    return LaunchDescription([
        declare_eagleye_area_arg,
        declare_switching_area_arg,
        area_node,
    ])
