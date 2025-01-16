import subprocess
import json
import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from chassis_control.msg import SetVelocity
import time

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe("gamepad")

def print_gamepad_data(data):    
    print('\033[F'*5, end='')
    print(' J0: ', ["{:+.2f}".format(i) for i in data["axes"][0:3]], '\033[K')
    print(' J1: ', ["{:+.2f}".format(i) for i in data["axes"][3:6]], '\033[K')
    print("Hat: ", data["hat"], '\033[K')
    button_str = f"{data['buttons']:011b}"
    print("Buttons: ", button_str, '\033[K')

shake = False
last_message = None
def on_message(client, userdata, msg):
    global shake
    global last_message
    payload = json.loads(msg.payload.decode('utf-8'))
    if msg.topic == 'gamepad':
        print_gamepad_data(payload)
        shake = payload['buttons'] & 0b00000000001
        message = {
            "shake" : shake
        }
        if message != last_message:
            last_message = message
            client.publish("action", json.dumps(message), qos=1)
    else:
        print("Received unhandled message: ", msg.topic, payload)

    # send it to rospy

# Clear the screen
print("\033[2J\033[H", end='')

mqtt_broker_ip = "localhost"  # Since the broker is on the same device
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = lambda client, userdata, rc: print("Disconnected from MQTT broker with code " + str(rc))
client.connect(mqtt_broker_ip, 1883, 60)
client.loop_forever()


# class RospyController:
#     def __init__(self, node_name):
#         self.node_name = node_name
#         self.subscribers = []
#         self.publishers = []

#     def init_node(self):
#         import rospy
#         rospy.init_node(self.node_name)

#     def publish_data(self, topic, data):
#         import rospy
#         from std_msgs.msg import String
#         pub = rospy.Publisher(topic, String, queue_size=10)
#         rospy.loginfo(f"Publishing data to {topic}: {data}")
#         pub.publish(data)

#     def subscribe_to_topic(self, topic, callback):
#         import rospy
#         from std_msgs.msg import String
#         sub = rospy.Subscriber(topic, String, callback)
#         self.subscribers.append(sub)
#         rospy.loginfo(f"Subscribed to {topic}")




# rospy.init_node('car_control_demo', log_level=rospy.DEBUG)

# set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

# def main():
#     rospy.sleep(1)
#     set_velocity.publish(150, 90, 0)
#     rospy.sleep(2)
#     set_velocity.publish(0, 0, 0)# 停止运动
#     rospy.sleep(1)
#     set_velocity.publish(150, 270, 0)# 控制底盘后退，发布底盘控制消息,线速度150，方向角270，偏航角速度0(小于0，为顺时针方向)
#     rospy.sleep(2)
#     set_velocity.publish(0, 0, 0)# 停止运动
#     rospy.sleep(1)
#     set_velocity.publish(0, 90, 0.1)# # 控制底盘逆时针旋转，发布底盘控制消息,线速度0，方向角90，偏航角速度0.1(小于0，为顺时针方向)
#     rospy.sleep(2)
#     set_velocity.publish(0, 0, 0)# 停止运动
#     rospy.sleep(1)
#     set_velocity.publish(150, 90, 0)# 控制底盘前进，发布底盘控制消息,线速度150，方向角90，偏航角速度0(小于0，为顺时针方向)
#     rospy.sleep(2)
#     set_velocity.publish(0, 0, 0)# 停止运动
#     rospy.sleep(1)
        
          
# if __name__ == '__main__':
#     main()
#     set_velocity.publish(0, 0, 0)  # 清除底盘控制消息
