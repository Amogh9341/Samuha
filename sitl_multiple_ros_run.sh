#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default sitl_gazebo-classic'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 10 -m iris
"""
	yeah was sourcing issue, now I got to conclude that ROS2 is the only way to go it
	is not possible to make it work with MAVLink cuz PX4 does no support changing udp ports of MAVLink.
	There is no way to have multiple PX4 instances sending MAVLink data to different ports, atleast not
	something that is easy enough to be worth the effort of making it work. Maybe in future.
"""

function cleanup() {
	pkill -x px4 || true
	pkill gzclient || true
	pkill gzserver || true
	pkill -f MicroXRCEAgent || true
}

function cleanup_all() {
    echo "[CLEANUP] Stopping PX4 + Gazebo processes..."

    # Kill PX4 SITL instances
    pkill -9 -f px4 || true

    # Kill Gazebo Classic server & client
    pkill -9 -f gzserver || true
    pkill -9 -f gzclient || true

    # Kill ROS 2 Gazebo bridge / XRCE agent if running
    pkill -9 -f MicroXRCEAgent || true
    pkill -9 -f micrortps_agent || true

    echo "[CLEANUP] Done."
}

function spawn_model() {
	MODEL=$1
	N=$2 # PX4 instance number
	X=${3:-0.0}
	Y=${4:-$((3*N))}

	SUPPORTED_MODELS=("iris" "plane" "standard_vtol" "rover" "r1_rover" "typhoon_h480")
	if [[ " ${SUPPORTED_MODELS[*]} " != *" ${MODEL} "* ]]; then
		echo "ERROR: Vehicle model ${MODEL} not supported!"
		echo "Supported models: ${SUPPORTED_MODELS[*]}"
		exit 1
	fi

	# Instance-specific paths
	working_dir="$build_path/rootfs/$N"
	mkdir -p "$working_dir"


	# PX4 + DDS parameters
	MAVLINK_TCP_PORT=$((4560 + N))
	MAVLINK_UDP_PORT=$((14560 + N))
	MAVLINK_ID=$((1 + N))
	GST_PORT=$((5600 + N))
	CAM_PORT=$((14530 + N))

	# DDS (ROS 2) isolation
	DDS_CLIENT_KEY=$((100 + N))
	ROS_NAMESPACE="px4_$N"

	# Only fix namespace for FIRST instance
	if [ "$N" -eq 0 ]; then
		DDS_NAMESPACE="px4_0"
		# PX4_UXRCE_DDS_NS=${DDS_NAMESPACE}
	else
		DDS_NAMESPACE=""
	fi


	pushd "$working_dir" &>/dev/null
	echo "▶ Starting PX4 instance $N (namespace: ${ROS_NAMESPACE})"

	export MAV_SYS_ID=$((N+1))

	# Start PX4 SITL
	if [ "$N" -eq 0 ]; then
		PX4_UXRCE_DDS_CLIENT_KEY=${DDS_CLIENT_KEY} \
		PX4_UXRCE_DDS_NS=${DDS_NAMESPACE} \
		PX4_SIM_MODEL=gazebo-classic_${MODEL} \
		PX4_SHELL=/tmp/px4-${N} \
		PX4_GZ_WORLD=default \
		$build_path/bin/px4 -i $N -d "$build_path/etc" >out.log 2>err.log &
	else
		PX4_UXRCE_DDS_CLIENT_KEY=${DDS_CLIENT_KEY} \
		PX4_SIM_MODEL=gazebo-classic_${MODEL} \
		PX4_SHELL=/tmp/px4-${N} \
		PX4_GZ_WORLD=default \
		$build_path/bin/px4 -i $N -d "$build_path/etc" >out.log 2>err.log &
	fi


# 	# Give PX4 time to boot
# Changing port does not work with current PX4 version in Jan 2026
# 	sleep 1

# 	echo "▶ Configuring MAVLink UDP for PX4_${N} on port ${MAVLINK_UDP_PORT}"

# 	socat - UNIX-CONNECT:/tmp/px4-${N} <<EOF
# 	mavlink stop-all
# 	mavlink start -u ${MAVLINK_UDP_PORT} -r 4000000 -b
# EOF


	# Generate SDF using jinja
	python3 \
		${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
		${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${MODEL}/${MODEL}.sdf.jinja \
		${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
		--mavlink_tcp_port ${MAVLINK_TCP_PORT} \
		--mavlink_udp_port ${MAVLINK_UDP_PORT} \
		--mavlink_id ${MAVLINK_ID} \
		--gst_udp_port ${GST_PORT} \
		--video_uri ${GST_PORT} \
		--mavlink_cam_udp_port ${CAM_PORT} \
		--generate_ros_models True\
		--output-file /tmp/${MODEL}_${N}.sdf

	echo "▶ Spawning ${MODEL}_${N} at (${X}, ${Y})"

	gz model \
		--spawn-file=/tmp/${MODEL}_${N}.sdf.last_generated \
		--model-name=${MODEL}_${N} \
		-x ${X} -y ${Y} -z 0.83


	popd &>/dev/null
}


cleanup_all

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	exit 1
fi

while getopts n:m:w:s:t:l: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
		l) LABEL=_${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
target=${TARGET:=px4_sitl_default}
vehicle_model=${VEHICLE_MODEL:="iris"}
export PX4_SIM_MODEL=gazebo-classic_${vehicle_model}

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../.."

build_path=${src_path}/build/${target}
mavlink_udp_port=14560
mavlink_tcp_port=4560

echo "killing running instances"
cleanup_all

sleep 1
# Source ROS2 and gazebo setup files
source /opt/ros/humble/setup.bash
source ${src_path}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

# To use gazebo_ros ROS2 plugins
# Checks if ROS_VERSION environment variable is set and non-empty (-n flag)
# AND if ROS_VERSION equals "2" (ROS 2 is installed and active)
# This environment variable is typically set by ROS setup scripts (e.g., /opt/ros/<distro>/setup.bash)
# when ROS is sourced in the current shell session
if [[ -n "$ROS_VERSION" ]] && [ "$ROS_VERSION" == "2" ]; then
	ros_args="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
else
	ros_args=""
fi

echo "Starting gazebo"
gzserver ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/${world}.world --verbose $ros_args &
sleep 5

n=0
if [ -z ${SCRIPT} ]; then
	if [ $num_vehicles -gt 255 ]
	then
		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
		exit 1
	fi

	while [ $n -lt $num_vehicles ]; do
		spawn_model ${vehicle_model} $n
		n=$(($n + 1))
	done
else
	IFS=,
	for target in ${SCRIPT}; do
		target="$(echo "$target" | tr -d ' ')" #Remove spaces
		target_vehicle=$(echo $target | cut -f1 -d:)
		target_number=$(echo $target | cut -f2 -d:)
		target_x=$(echo $target | cut -f3 -d:)
		target_y=$(echo $target | cut -f4 -d:)

		if [ $n -gt 255 ]
		then
			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi

		m=0
		while [ $m -lt ${target_number} ]; do
			export PX4_SIM_MODEL=gazebo-classic_${target_vehicle}
			spawn_model ${target_vehicle}${LABEL} $n $target_x $target_y
			m=$(($m + 1))
			n=$(($n + 1))
		done
	done

fi
trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient
