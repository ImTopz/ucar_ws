import rospy



print("staring!")
nav_state = rospy.get_param('nav_state')
while True:
    print(nav_state)