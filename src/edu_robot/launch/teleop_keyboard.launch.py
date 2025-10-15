from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch the standard teleop_twist_keyboard node.
    
    This node converts keyboard inputs (via the terminal where it is launched) 
    into geometry_msgs/Twist messages on the '/cmd_vel' topic.
    """
    
    # ----------------------------------------------------
    # 1. teleop_twist_keyboard ノードの定義
    # ----------------------------------------------------
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        
        # 💡 修正ポイント: prefix を設定して、新しいターミナルでノードを実行する
        # 'xterm -e' は、軽量で多くの環境で利用可能なターミナルエミュレータです。
        # 環境に応じて 'gnome-terminal -x' や 'konsole -e' などに変更できます。
        # Jetson環境であれば 'xterm' がインストールされている可能性が高いです。
        prefix=['xterm -e'], 
        
        # ノードがTwistメッセージをパブリッシュするトピックのリマップ設定 (オプション)
        # remappings=[
        #     ('/cmd_vel', '/edu_robot/cmd_vel'),
        # ],
    )
    
    # ----------------------------------------------------
    # LaunchDescriptionの組み立て
    # ----------------------------------------------------
    return LaunchDescription([
        teleop_keyboard_node,
    ])
