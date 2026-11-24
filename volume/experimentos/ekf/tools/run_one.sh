#!/bin/bash
# Uso: run_one.sh <nombre> <only_prediction true|false> <twist> <duracion_s>
# Corre UNA corrida con el protocolo: Stop -> nodos -> Play -> esperar init -> grabar + mover -> Stop
source /opt/ros/humble/setup.bash; source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=1
NAME=$1; ONLYPRED=$2; TWIST=$3; DUR=$4
TOOLS=/root/ros2_ws/src/robotica/experimentos/ekf/tools
OUT=/root/ros2_ws/src/robotica/experimentos/ekf/runs/$NAME; rm -rf $OUT; mkdir -p $OUT
python3 $TOOLS/simctl.py stop >/dev/null; bash $TOOLS/killnodes.sh
P="--ros-args -p use_sim_time:=true"
ros2 run modelo_omnidireccional omni_odometry_node $P > $OUT/odometry.log 2>&1 &
ros2 run imu_laser landmark_detector $P -p laser_frame:=front_laser -p publish_robot_frame:=base_link_ekf > $OUT/detector.log 2>&1 &
ros2 run robmovil_ekf localizer $P -p only_prediction:=$ONLYPRED -p min_landmark_size:=2 > $OUT/localizer.log 2>&1 &
sleep 5
python3 $TOOLS/simctl.py start >/dev/null
for i in $(seq 1 40); do grep -q "EKF inicializado" $OUT/localizer.log && break; sleep 0.5; done
grep -q "EKF inicializado" $OUT/localizer.log || { echo "[$NAME] ERROR: el EKF no se inicializo"; bash $TOOLS/killnodes.sh; exit 1; }
ros2 bag record -o $OUT/bag /pose /robot/odometry /robot/ground_truth /tf /tf_static /landmarks /clock > $OUT/bag.log 2>&1 &
python3 $TOOLS/sampler.py $OUT/poses.csv $((DUR + 4)) &
SAMPLER=$!
sleep 2
timeout $DUR ros2 topic pub -r 10 /robot/cmd_vel geometry_msgs/msg/Twist "$TWIST" > /dev/null 2>&1
ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
wait $SAMPLER
pkill -INT -f "ros2 bag record"; sleep 2
python3 $TOOLS/simctl.py stop >/dev/null; bash $TOOLS/killnodes.sh
echo "[$NAME] ok: $(($(wc -l < $OUT/poses.csv) - 1)) muestras"
