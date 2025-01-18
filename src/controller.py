import paho.mqtt.client as mqtt
import pygame
import json
import socket
import struct
import threading
import curses
from gamepad import GamepadController

controller = None
client = None

def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code " + str(rc))
    client.subscribe("action")
    message = {
        "remote": local_ip()
    }
    print('sending connect message: ', message)
    client.publish("connect", json.dumps(message), qos=1)

shake = False
def on_message(client, userdata, msg):
    global shake
    global controller
    data = json.loads(msg.payload.decode('utf-8'))
    if msg.topic == 'status':
        print('status: ', data)
    print(data)

def mqtt_loop():
    client.loop_forever()



def print_gamepad_data(stdscr, data):
    stdscr.addstr(0, 0, ' J0: ' + str(["{:+.2f}".format(i) for i in data["axes"][0:3]]))
    stdscr.addstr(1, 0, ' J1: ' + str(["{:+.2f}".format(i) for i in data["axes"][3:6]]))
    print("Hat: ", data["hat"], '\033[K')
    button_str = f"{data['buttons']:011b}"
    stdscr.addstr("Buttons: ", button_str, '\033[K')
    stdscr.refresh()

def pygame_loop():
    pygame.init()
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
        
                
def main():
    global client
    global controller

    curses.curs_set(0)
    stdscr.clear()
    stdscr.refresh()

    print('local ip: ', local_ip())
    mqtt_broker_ip = "192.168.1.21"  # Replace with the IP address of your Raspberry Pi
    # mqtt_broker_ip = "localhost" 
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = lambda client, userdata, rc: print("Disconnected from MQTT broker with code " + str(rc))
    client.connect(mqtt_broker_ip, 1883, 60)
    mqtt_thread = threading.Thread(target=mqtt_loop)
    mqtt_thread.start()

    controller = GamepadController()
    controller.init_gamepad()
    
    pygame_thread = threading.Thread(target=pygame_loop)
    pygame_thread.start()


    stdscr.getch()  # Wait for user input

if __name__ == "__main__":
    curses.wrapper(main)

print("\033[?25h", end='') # show the cursor