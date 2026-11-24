#!/bin/bash
# Corre todas las configuraciones x movimientos. Al final restaura localizer_ekf.cpp original.
TOOLS=/root/ros2_ws/src/robotica/experimentos/ekf/tools
source /opt/ros/humble/setup.bash
build() { python3 $TOOLS/set_qr.py $1 $2 && (cd /root/ros2_ws && colcon build --packages-select robmovil_ekf > /tmp/build.log 2>&1) || { echo "BUILD FALLO"; exit 1; }; }
declare -A MOV=( [rot]="{angular: {z: 0.4}}|30" [curva]="{linear: {x: 0.15}, angular: {z: 0.2}}|40" [rotrapida]="{angular: {z: 1.5}}|20" [orbita]="{linear: {y: -0.5}, angular: {z: -0.4}}|31" [orbitarapida]="{linear: {y: -0.8}, angular: {z: -0.64}}|30" [circulo]="{linear: {x: -0.4, y: 0.4}, angular: {z: 0.2}}|32" )
MOVS=${MOVS:-"rot curva rotrapida"}
# nombre  factorQ factorR only_prediction
CONFIGS="soloPred,1,1,true base,1,1,false q10,10,1,false q100,100,1,false qdiv10,0.1,1,false r10,1,10,false r100,1,100,false rdiv10,1,0.1,false"
for c in $CONFIGS; do
  IFS=, read NAME FQ FR OP <<< "$c"
  build $FQ $FR
  for m in $MOVS; do
    IFS='|' read TW DUR <<< "${MOV[$m]}"
    bash $TOOLS/run_one.sh ${m}_${NAME} $OP "$TW" $DUR
  done
done
python3 $TOOLS/set_qr.py 1 1 && (cd /root/ros2_ws && colcon build --packages-select robmovil_ekf > /tmp/build.log 2>&1)
chown -R 1000:1000 /root/ros2_ws/src/robotica/experimentos
echo "TODO TERMINADO (localizer_ekf.cpp restaurado a valores originales)"
