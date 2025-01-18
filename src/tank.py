import sys,os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import json
import paho.mqtt.client as mqtt
from motor_driver import MotorDriver

motor_driver = MotorDriver()

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

def deadband(value, threshold=0.1):
    # Apply a deadband to the joystick, and fade in from the deadband
    if abs(value) < threshold:
        return 0
    else:
        return (abs(value) - threshold) / (1 - threshold) * value / abs(value) 

def track_drive(payload):
    joystick = payload["axes"][0:2]
    # unpack joystick[[0:1] to left and right track drive
    joy_x = deadband(joystick[0])
    joy_y = deadband(joystick[1])
    left = -(joy_y - joy_x)
    right = -(joy_y + joy_x)
    print("Left: ", left, "Right: ", right)
    motor_driver.set_tracks([int(left*255), int(right*255)])
    # set_velocity.publish(left, right, 0)

def battery_check(gamepad):
    if gamepad["buttons"] & 0b000000000010:
        battery = motor_driver.read_battery()
        message = {
            "battery": battery
        }
        client.publish("status", json.dumps(message))

shake = False
last_message = None
def on_message(client, userdata, msg):
    global shake
    global last_message
    payload = json.loads(msg.payload.decode('utf-8'))
    if msg.topic == "connect":
        print("Connected to remote at", payload["remote"])
    elif msg.topic == 'gamepad':
        print_gamepad_data(payload)
        track_drive(payload)
        battery_check(payload)
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
