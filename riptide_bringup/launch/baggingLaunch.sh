#acceptable choices vision, sensors, mapping, diagnostics, competition, all, test
#./baggingLaunch.sh [bagging choice]

#To run must install
    #ros-galactic-ros2bag
    #ros-galactic-rosbag2-storage-default-plugins
#And maybe install - if it works
    #ros-galactic-rosbag2-converter-default-plugins

#You can ignore the firmware error - probably
#All bags record voltage!

echo "Starting Ros2 Bagging Node!!"
date=$(date +'%m_%d_%Y__%H_%M_%S')


case $1 in 
    #bag vision
    "vision") 
        echo "Bagging Vision!"
        saveName="../Bags/vision__$date"

        #launch the vision bagging
        ros2 bag record --include-hidden-topics --max-bag-duration 15 -o $saveName \
            /rosout_agg \
            stereo/left/image_raw/compressed \
            stereo/left/camera_info \
            stereo/right/image_raw/compressed \
            stereo/right/camera_info \
            /tempest/state/electrical
        ;;

    #bag sensors
    "sensors")
        echo "Bagging Sensors!"
        saveName="../Bags/sensors__$date"

        #launch sensor bagging
        ros2 bag record --include-hidden-topics -d 15 -o $saveName \
            /rosout_agg \
            /tempest/dvl/twist \
            /tempest/dvl/status \
            /tempest/dvl/dvl_sonar0 \
            /tempest/dvl/dvl_sonar1 \
            /tempest/dvl/dvl_sonar2 \
            /tempest/dvl/dvl_sonar3 \
            /tempest/depth/raw \
            /tempest/depth/pose \
            /tempest/imu/data \
            /tf \
            /tempest/state/electrical

        ;; 

    #  competition
    "competition")
        echo "Bagging the competition profile"
        saveName="../Bags/competition__$date"
        ros2 bag record --include-hidden-topics --max-bag-duration 15 -o $saveName \
            /rosout_agg \
            odometry/filtered \
            stereo/left_raw/image_raw \
            /tempest/state/electrical
            dvl/twist \
            dvl_twist \
            dvl_sonar0 \
            dvl_sonar1 \
            dvl_sonar2 \
            vl_sonar3 \
            depth/raw \
            depth/pose \
            imu/data \
            /tf
        ;;

    #bag mapping
    "mapping")
        echo "Bagging Mapping"
        saveName="../Bags/mapping__$date"

        #launch mapping bagging
        ros2 bag record --include-hidden-topics -d 15 -o $saveName \
            /rosout_agg \
            /tempest/mapping/cutie \
            /tempest/mapping/TommyGun \
            /tempest/mapping/gman \
            /tempest/mapping/bootlegger \
            /tempest/mapping/torpedoBootlegger \
            /tempest/mapping/torpedoGman \
            /tempest/mapping/axe \
            /tempest/mapping/cash \
            /tempest/mapping/BinBarrel \
            /tempest/mapping/BinPhone \
            /tempest/mapping/badge \
            /tempest/dope/detected_objects \
            /tempest/depth/pose \
            /tf \
            /tempest/state/electrical

        ;;

    #bagging diagnostics
    "diagnostics")
        echo "Bagging Diagnostics"
        saveName="../Bags/diagnostics__$date"
        
        #launch diagnostics bagging
        ros2 bag record -d 15 --include-hidden-topics -o $saveName \
            /rosout_agg \
            /diagnostics \
            /diagnostics_agg \
            /diagnostics_toplevel_state \
            /tempest/state/electrical

        ;;

    #bag all topics - not recommended
    "all")
        echo "Bagging All Topics - Prepare your hardrive/SSD!!!!"
        saveName="../Bags/all__$date"

        #launch all bagging
        ros2 bag record --include-hidden-topics -d 15 -o $saveName -a
    ;;

    #catch all
    "")
        echo "No topic entered! Please enter either vision, diagnostics, mapping, sensors or all."
        ;;


    #doesn't bag anything - just for testing ;)
    "test")
        echo "Testing Mode!"
        date=$(date +'%m_%d_%Y__%H_%M_%S')
        echo "test$date"
    ;;

esac
