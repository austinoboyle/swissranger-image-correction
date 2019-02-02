# SwissRanger Correction ROS Node

## Set Up

### Usual ROS Stuff

```bash
source /opt/ros/kinetic/setup.bash
git clone https://github.com/austinoboyle/swissranger-image-correction.git
cd swissranger-image-correction
catkin_make
source ./devel/setup.bash
```

### Running the node

```
rosrun repub repub_approx.py
```

### Testing with sample data

```
# Run in a separate terminal instance, can observe results in something like rqt
rosbag --loop data/sr_test_00.bag
```
