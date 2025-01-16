#https://drive.google.com/file/d/1-3J3A1OdviXSMbrbsaH2JZLdyvB2ZweY/view?usp=drive_link
import sys
import rospy
from chassis_control.msg import SetVelocity

rospy.init_node('car_control_demo', log_level=rospy.DEBUG)

set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

def main():
    rospy.sleep(1)
    set_velocity.publish(150, 90, 0)
    rospy.sleep(2)
    set_velocity.publish(0, 0, 0)# 停止运动
    rospy.sleep(1)
    set_velocity.publish(150, 270, 0)# 控制底盘后退，发布底盘控制消息,线速度150，方向角270，偏航角速度0(小于0，为顺时针方向)
    rospy.sleep(2)
    set_velocity.publish(0, 0, 0)# 停止运动
    rospy.sleep(1)
    set_velocity.publish(0, 90, 0.1)# # 控制底盘逆时针旋转，发布底盘控制消息,线速度0，方向角90，偏航角速度0.1(小于0，为顺时针方向)
    rospy.sleep(2)
    set_velocity.publish(0, 0, 0)# 停止运动
    rospy.sleep(1)
    set_velocity.publish(150, 90, 0)# 控制底盘前进，发布底盘控制消息,线速度150，方向角90，偏航角速度0(小于0，为顺时针方向)
    rospy.sleep(2)
    set_velocity.publish(0, 0, 0)# 停止运动
    rospy.sleep(1)
        
          
if __name__ == '__main__':
    main()
    set_velocity.publish(0, 0, 0)  # 清除底盘控制消息
