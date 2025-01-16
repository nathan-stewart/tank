import paho.mqtt.client as mqtt
import pygame
import json
import struct
import threading
from gamepad import GamepadController

controller = None
client = None
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe("action")

shake = False
def on_message(client, userdata, msg):
    global shake
    global controller
    data = json.loads(msg.payload.decode('utf-8'))
    if msg.topic == 'action':
        if shake != data['shake']:
            shake = data['shake']
            controller.rumble(shake)
    print(data)

def mqtt_loop():
    client.loop_forever()

def clear_screen():
    print("\033[?25l", end='') # hide the cursor
    print('\033[2J\033[H', end='')  # Clear the screen and move the cursor to the top-left corner

def print_gamepad_data(data):    
    print('\033[F'*5, end='')
    print(' J0: ', ["{:+.2f}".format(i) for i in data["axes"][0:3]], '\033[K')
    print(' J1: ', ["{:+.2f}".format(i) for i in data["axes"][3:6]], '\033[K')
    print("Hat: ", data["hat"], '\033[K')
    button_str = f"{data['buttons']:011b}"
    print("Buttons: ", button_str, '\033[K')

def main():
    global client
    global controller
    # mqtt_broker_ip = "192.168.1.21"  # Replace with the IP address of your Raspberry Pi
    mqtt_broker_ip = "localhost" 
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = lambda client, userdata, rc: print("Disconnected from MQTT broker with code " + str(rc))
    client.connect(mqtt_broker_ip, 1883, 60)
    mqtt_thread = threading.Thread(target=mqtt_loop)
    mqtt_thread.start()

    controller = GamepadController()
    controller.init_gamepad()
    clear_screen()

    last_message = None
    while True:
        for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        return
        
        axes, hat, buttons  = controller.read_input()
        message = {
            "axes": axes,
            "hat": hat,
            "buttons": buttons
        }
        if message != last_message:
            last_message = message 
            print_gamepad_data(message)
            if client:
                client.publish("gamepad", json.dumps(message), qos=1)
            
if __name__ == "__main__":
    main()

print("\033[?25h", end='') # show the cursor