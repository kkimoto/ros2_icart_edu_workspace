
x=-0.6
y=-3.5
a=0.0

echo initial position $x $y $a

./myscripts/spur_set_pos $x $y $a
sleep 1
python3 ./src/edu_robot/bin/amcl_initial_pose.py $x $y $a
