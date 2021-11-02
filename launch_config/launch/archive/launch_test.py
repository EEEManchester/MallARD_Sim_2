#! /usr/bin/env python

import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/gyal/mallard_ws/src/mallard_sim/launch_config/launch/mallard_gazebo.launch"])
launch.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
launch.shutdown()