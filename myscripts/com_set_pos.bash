
echo start
until ros2 lifecycle get /amcl | grep -q active; do
  echo wait
  sleep 0.2
done

# ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "
# header:
#   frame_id: 'map'
# pose:
#   pose:
#     position:
#       x: 0.0
#       y: -3.73
#       z: 0.0
#     orientation:
#       z: 0.0
#       w: 1.0
#   covariance: [0.25, 0, 0, 0, 0, 0,
#                0, 0.25, 0, 0, 0, 0,
#                0, 0, 0.01, 0, 0, 0,
#                0, 0, 0, 0.01, 0, 0,
#                0, 0, 0, 0, 0.01, 0,
#                0, 0, 0, 0, 0, 0.01]"

/home/kimoto/ros2_workspace/lib/yp-spur/build/samples/spur_set_pos  0 -3.73 0

echo done
