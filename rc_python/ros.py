import rospy
from geometry_msgs.msg import Vector3

class Ros():
    '''
    UESTC 2018 Robocon Team
    ROS Package
    '''
    def __init__(self, dash):
        self._dash = dash
        self._gyroData = Vector3()
        rospy.init_node('rc2018', anonymous=True)
        rospy.Subscriber("dashAt", Vector3, self.__dashAtCB)
        rospy.Subscriber("dashBy", Vector3, self.__dashByCB)
        rospy.Subscriber("dashTo", Vector3, self.__dashToCB)
        self._gyroPub = rospy.Publisher('gyro', Vector3, queue_size=1)
        self.__loop()
    
    def __loop(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.__query()
            self._gyroPub.publish(self._gyroData)
            rate.sleep()
    
    def __dashAtCB(self, speed):
        self._dash.at(speed[0], speed[1], speed[2])
        self._dash.unlock()
    
    def __dashByCB(self, increment):
        self._dash.by(increment[0], increment[1], increment[2])
        self._dash.unlock()
    
    def __dashToCB(self, goal):
        self._dash.to(goal[0], goal[1], goal[2])
        self._dash.unlock()
    
    def __query(self):
        self._gyroData.x = self._dash.position[0]
        self._gyroData.y = self._dash.position[1]
        self._gyroData.z = self._dash.position[2]
