echo "Building ROS nodes"

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/mjlee/repositories/external_ros/ORB_SLAM2/Examples/ROS

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
