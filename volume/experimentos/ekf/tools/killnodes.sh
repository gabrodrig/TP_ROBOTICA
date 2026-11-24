#!/bin/bash
# Mata los nodos de un experimento (no toca Coppelia ni RViz)
for p in "ros2 launch" "ros2 bag record" landmark_detector omni_odometry_node lib/robmovil_ekf/localizer \
         static_transform_publisher keys_to_twist "lib/keyboard/keyboard" "topic pub" sampler.py; do
  pgrep -f -- "$p" | grep -v "^$$\$" | xargs -r kill 2>/dev/null
done
sleep 2
