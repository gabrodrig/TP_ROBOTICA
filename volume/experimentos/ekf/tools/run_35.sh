#!/bin/bash
# Punto 3.5: corre el cuadrado a lazo cerrado con lazo_cerrado.launch.py y graba todo.
# Uso: run_35.sh <nombre> <frame_feedback: base_link_ekf | base_link> <duracion_s>
source /opt/ros/humble/setup.bash; source /root/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=1
NAME=$1; FRAME=$2; DUR=$3
TOOLS=/root/ros2_ws/src/robotica/experimentos/ekf/tools
CTRL=/root/ros2_ws/src/robotica/lazo_cerrado/src/KinematicPositionController.cpp
OUT=/root/ros2_ws/src/robotica/experimentos/ekf/runs/$NAME; rm -rf $OUT; mkdir -p $OUT
build() { (cd /root/ros2_ws && colcon build --packages-select lazo_cerrado > /tmp/build35.log 2>&1) || { echo "BUILD FALLO"; exit 1; }; }
if [ "$FRAME" != "base_link_ekf" ]; then      # cambio temporal del feedback, se restaura al final
  cp $CTRL /tmp/ctrl_backup.cpp
  sed -i "s#lookupTransform(\"odom\", \"base_link_ekf\"#lookupTransform(\"odom\", \"$FRAME\"#" $CTRL
  grep -q "lookupTransform(\"odom\", \"$FRAME\"" $CTRL || { echo "no pude cambiar el frame"; exit 1; }
  build
fi
python3 $TOOLS/simctl.py stop >/dev/null; bash $TOOLS/killnodes.sh
cd $OUT && ros2 launch lazo_cerrado lazo_cerrado.launch.py > $OUT/launch.log 2>&1 &
# reinicia el RViz del usuario (punto3_5.rviz) para que los rastros arranquen vacios; de esa ventana salen las capturas
pkill -f "rviz2 -d" ; sleep 1
rviz2 -d /root/ros2_ws/src/robotica/punto3_5.rviz --ros-args -p use_sim_time:=true > $OUT/rviz.log 2>&1 &
sleep 8
python3 $TOOLS/simctl.py start >/dev/null
for i in $(seq 1 40); do grep -q "EKF inicializado" $OUT/launch.log && break; sleep 0.5; done
grep -q "EKF inicializado" $OUT/launch.log || echo "[$NAME] AVISO: no vi 'EKF inicializado'"
ros2 bag record -o $OUT/bag /pose /robot/odometry /robot/ground_truth /tf /tf_static /landmarks /clock \
  /goal_pose /robot/cmd_vel /ground_truth/target_path > $OUT/bag.log 2>&1 &
mkdir -p $OUT/capturas
capture() {  # captura solo el area 3D del RViz (subventana de render) y le agrega la leyenda
  W=$(xwininfo -root -tree 2>/dev/null | grep -A3 'punto3_5.rviz' | tail -n +2 | grep -oE '0x[0-9a-f]+' | head -1)
  [ -n "$W" ] && xwd -id $W -silent | convert xwd:- -gravity south -background '#1e1e1e' -splice 0x34 -fill white -pointsize 15 \
    -annotate +0+8 "blanco: cuadrado pedido   celeste: recorrido real   verde: odometria   naranja: elipse EKF (x3)   magenta: goal   amarillo: postes" \
    $OUT/capturas/$1.png
}
( for i in $(seq 1 $((DUR / 20))); do sleep 20; capture "t$(printf %03d $((i * 20)))s"; done ) &
CAPS=$!
python3 $TOOLS/sampler.py $OUT/poses.csv $DUR
kill $CAPS 2>/dev/null; capture final
pkill -INT -f "ros2 bag record"; sleep 2
python3 $TOOLS/simctl.py stop >/dev/null; bash $TOOLS/killnodes.sh
if [ "$FRAME" != "base_link_ekf" ]; then cp /tmp/ctrl_backup.cpp $CTRL; build; fi
chown -R 1000:1000 /root/ros2_ws/src/robotica/experimentos
echo "[$NAME] ok: $(($(wc -l < $OUT/poses.csv) - 1)) muestras, path: $(ls $OUT/poses_path.csv 2>/dev/null && echo si || echo NO)"
