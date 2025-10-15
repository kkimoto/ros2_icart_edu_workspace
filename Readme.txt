

1. joystick でロボットを動かすところまで

joy => [/joy] teleop_twist_joy => [/cmd_vel] => icart_mini_driver => [device /dev/ttyp_ACM0] yp-spur

myscripts 各種スクリプト（ほとんど1行）
  com_build_all.bash          すべてビルドする。
  com_build_edu_roboto.bash   edu_robot package だけビルドする。
  com_clearn_all.bash         build と install を削除する。
  lifecycle_joystick.bash     joystick          のライフサイクルの管理
  lifecycle_icart.bash        icart_mini_driver のライフサイクルの管理
  lifecycle_yp_spur.bash      yp-spur           のライフサイクルの管理

統合ノード src/edu_robot
 setup.cfg
 setup.py
   Python のノードを起動するために entry_point 定義が必要。

ラッパーノード src/edu_robot/edu_robot
  life-cycle をサポートしない node に対して life-cycle 機能を付加するラッパーノード
    icart_lifecycle_wrapper.py
    teleop_lifecycle_wrapper.py
    yp_spur_lifecycle_wrapper.py

Launcher src/edu_robot/launch/
  yp_spur.launch.py
  icart.launch.py
  teleop.launch.py

パラメータ定義ファイル
  一つのファイルにまとめました。
  src/edu_roboto/config/params_default.yaml

動かしてみる
  ビルド
    cd <workspace>
    bash myscripts/com_build_all.bash
  起動
    source install/setup.bash
    ros launch icat_lifecycle_wrapper.py
    ros launch yp_spur_lifecycle_wrapper.py
    ros launch teleop_lifecycle_wrapper.py
  ライフサイクルでアクティベート
    bash myscripts/lifecycle_yp_spur.bash configure
    bash myscripts/lifecycle_yp_spur.bash activate
    bash myscripts/lifecycle_icart.bash configure
    bash myscripts/lifecycle_icart.bash activate
    bash myscripts/lifecycle_teleop.bash configure
    bash myscripts/lifecycle_teleop.bash activate

  終わる
    bash myscripts/lifecycle_yp_spur.bash deactivate
    bash myscripts/lifecycle_yp_spur.bash cleanup
    bash myscripts/lifecycle_yp_spur.bash shutdown
    bash myscripts/lifecycle_icart.bash deactivate
    bash myscripts/lifecycle_icart.bash cleanup
    bash myscripts/lifecycle_icart.bash shutdown
    bash myscripts/lifecycle_teleop.bash deactivate
    bash myscripts/lifecycle_teleop.bash cleanup
    bash myscripts/lifecycle_teleop.bash shutdown
