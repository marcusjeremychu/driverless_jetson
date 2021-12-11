#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from autonomous_system.msg import external_input

MANUAL_MISSION = False
AUTONOMOUS_MISSION = False
ASMS = False
GO = False
BRAKE_RELEASE = False
EBS_activate = True
MISSION_FINISHED = False
V0 = False
TRACTIVE_SYSTEM = False
R2D = False
ASB_check = False


def callback(data):
    global MANUAL_MISSION, AUTONOMOUS_MISSION, ASMS, GO, BRAKE_RELEASE, EBS_activate, MISSION_FINISHED, V0, TRACTIVE_SYSTEM, R2D, ASB_check
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    MANUAL_MISSION = data.MANUAL_MISSION
    rospy.loginfo(MANUAL_MISSION)
    AUTONOMOUS_MISSION = data.AUTONOMOUS_MISSION
    ASMS = data.ASMS
    GO = data.GO
    BRAKE_RELEASE = data.BRAKE_RELEASE
    EBS_activate = data.EBS_activate
    MISSION_FINISHED = data.MISSION_FINISHED
    V0 = data.V0
    TRACTIVE_SYSTEM = data.TRACTIVE_SYSTEM
    R2D = data.R2D
    ASB_check = data.ASB_check


# rostopic pub -1 /FSM_ext_input autonomous_system/external_input -- false false false false false false false false false false false

def run_FSM():
    pub = rospy.Publisher('AS_State', String, queue_size=1)
    rospy.init_node('FSM', anonymous=False)

    rate = rospy.Rate(10) # 100hz

    rospy.Subscriber("FSM_ext_input", external_input, callback)

    while not rospy.is_shutdown():
        #rospy.loginfo("loop manual mission %s", MANUAL_MISSION)
        if (EBS_activate): #EBS activated
            if ((MANUAL_MISSION or AUTONOMOUS_MISSION) and ASMS and ASB_check and TRACTIVE_SYSTEM):
                if (R2D):
                    pub.publish("AS Driving")
                else:
                    if (BRAKE_RELEASE):
                        pub.publish("AS Off")
                    else:
                        pub.publish("AS Ready")
            else:
                pub.publish("AS Off")
        else: #EBS not activated
            if (MISSION_FINISHED and V0):
                pub.publish("AS Finished")
            else:
                pub.publish("AS Emergency")
        
        rate.sleep()

    

    # pub = rospy.Publisher('FSM_State', String, queue_size=10)
    # rospy.init_node('run_FSM', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        run_FSM()
    except rospy.ROSInterruptException:
        pass