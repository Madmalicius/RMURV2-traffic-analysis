
# Getting started  - Ex2
Download an example dataset.
```
cd src/traffic_analysis_from_drones/data
pipenv install
make get_data Note: install youtube-dl before this can be done("pip install youtube-dl")
```

Get back to the main directory (`traffic_analysis_shared`)
and fetch the submodule with the image stabilizer.
```
cd ../../..
git submodule init
git submodule update
```

Build and launch.
```
source /opt/ros/melodic/setup.bash
catkin build
source devel/setup.bash
roslaunch traffic_analysis_from_drones track_traffic.launch
```
The file ('car_tracker.py') isn't done. This file take the image from ('stabilized_frame') and mark the image with a red circle.

The output of this file is stable frame.

# Ex3
