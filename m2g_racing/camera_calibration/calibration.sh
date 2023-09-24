


# rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.1 image:=/airsim_node/drone_1/front_left/Scene camera:/camera --no-service-check



rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.108 right:=/airsim_node/drone_1/front_right/Scene left:=/airsim_node/drone_1/front_left/Scene left_camera:=//airsim_node/drone_1/front_left right_camera:=/airsim_node/drone_1/front_right --no-service-check
