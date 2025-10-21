

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

2. tf の static transform 

  arno2b_ws と同じもの
  src/edu_robot/launch/static_transforms.launch.py 

  起動
    ros2 launch edu_robot static_transforms.launch.py 



3. 2D lidar

  準備
    cd src
    git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
    rosdep update
    rosdep install -i --from-paths urg_node2
    cd ..
    bash myscripts/com_build_all.bash

  launch file (起動後はinactive)
    src/edu_robot/launch/urg_node2.launch.py

  ライフサイクルの管理 (2台同時)
    lifecycle_urg_node2.bash

4. laser_scan_merger

  準備
    cd src/
    git clone https://github.com/BruceChanJianLe/laser_scan_merger.git
    rosdep update
    rosdep install -i --from-paths laser_scan_merger
    cd ..
    bash myscripts/com_build_all.bash

  ライフサイクル管理はしない
    lanuch file
      src/edu_robot/launch/laser_scan_merger.launch.py 

  起動
    ros2 launch edu_robot laser_scan_merger.launch.py 

5. すべてを launch する。

  bash myscripts/000_launch_main_system.bash

  teleop は起動しない。teleop するときだけ起動する。

6. icart_mini_driver の /odom の扱い

  まだよくわからない
  spur ライブラリでリセットできる。

7. slam_toolbox

  Ubuntu パッケージをインストール
    sudo apt install ros-humble-slam-toolbox

  launch file
    src/edu_robot/launch/slam_sync.launch.py

8. 地図作成

  基本システムの起動
    bash myscripts/000_launch_main_system.bash 

  slam mapping 起動
    bash myscripts/001_start_slam.bash 

  gamepad でロボットを動かして地図を作る

  地図保存
    bash myscripts/002_save_map.bash 

    map/mymap.yaml, map/mymap.pgm ができる。

9. way point 作成

  map は map/map.yaml, map/map.pgm を使う。
  mymap からファイル名を変える。map.yaml 内にも pgm 指定があるので書き換える。

  map server と nav2-amcl をつかう。

  sudo apt install ros-humble-nav2-amcl

  パラメータ追加 edu_robot/config/params_default.yaml

  launch file
    edu_robot/launch/make_wp.launch.py

  新規作成
    src/edu_robot/edu_robot/make_wp_joy.py


  別途 teleop も必要
  bash myscripts/001b_teleop_only.bash
  bash myscripts/003_make_wp.bash
    ここでは teleop と map_server と amcl が起動する。

  way point を置く

    ros2 run edu_robot make_wp_joy <way-point-file-name>

---
