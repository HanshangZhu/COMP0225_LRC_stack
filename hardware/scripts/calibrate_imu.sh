pkill -9 -f cartographer; pkill -9 -f transform_everything; pkill -9 -f rviz2; pkill -9 -f "ros2 daemon"; sleep 2; ros2 daemon start; sleep 1
source /opt/ros/humble/setup.bash
source ~/COMP0225_LRC_stack/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name=\"enxc8a36240a4c7\" multicast=\"true\" />
  </Interfaces></General><Discovery><Peers>
  <Peer address=\"192.168.123.161\"/></Peers>
  <ParticipantIndex>auto</ParticipantIndex>
  </Discovery></Domain></CycloneDDS>"
export ROS_DOMAIN_ID=0
python3 ~/COMP0225_LRC_stack/calibrate_imu.py --phase both --duration 30
