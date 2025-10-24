ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: "map"
  },
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685389]
  }
}'
