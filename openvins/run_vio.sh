#!/bin/bash
# Usage: run_vio.sh <seq_dir> <out_traj> [speed] [tag]
set +u
SEQ=$1
OUT=$2
SPEED=${3:-1.0}
TAG=${4:-run}
E=${ROS_ENV:-$HOME/anaconda3/envs/fastlio2}
WS=${OPENVINS_WS:-$HOME/openvins_ws}
source $E/setup.bash
source $WS/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11331
export ROS_HOME=/tmp/ros_openvins_$TAG
mkdir -p "$ROS_HOME" "$(dirname "$OUT")"
roscore -p 11331 > /tmp/roscore_vio_$TAG.log 2>&1 &
RC=$!
sleep 3
rosrun ov_msckf run_subscribe_msckf __name:=ov_msckf _config_path:=$WS/config_osdar/estimator_config.yaml _verbosity:=INFO > /tmp/openvins_$TAG.log 2>&1 &
RV=$!
sleep 5
rosrun ov_eval pose_to_file _topic:=/ov_msckf/poseimu _topic_type:=PoseWithCovarianceStamped _output:="$OUT" > /tmp/pose2file_$TAG.log 2>&1 &
RP=$!
sleep 2
$E/bin/python $WS/osdar_vio_player.py _seq:="$SEQ" _speed:=$SPEED > /tmp/player_vio_$TAG.log 2>&1
sleep 2
kill $RP $RV $RC 2>/dev/null || true
sleep 2
pkill -f "run_subscribe_msckf" 2>/dev/null || true
pkill -f "pose_to_file" 2>/dev/null || true
pkill -f "rosmaster --core -p 11331" 2>/dev/null || true
pkill -f "roscore -p 11331" 2>/dev/null || true
wait 2>/dev/null || true
echo "done $TAG"
