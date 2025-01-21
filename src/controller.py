import paho.mqtt.client as mqtt
import pygame
import json
import socket
import threading
import curses
import logging
import configparser
from gamepad import GamepadController

config = configparser.ConfigParser()
config.read('config.ini')
mqtt_server = config['DEFAULT']['mqtt_broker_address']
mqtt_port = int(config['DEFAULT']['mqtt_port'])
mqtt_keepalive = int(config['DEFAULT']['mqtt_keepalive'])
log_file = config['DEFAULT']['log_file']

logging.basicConfig(filename=log_file, level=logging.DEBUG, format='%(asctime)s - %(message)s')

running_lock = threading.Lock()
running_lock = threading.Lock()
mqtt_lock = threading.Lock()
mqtt_ready = threading.Condition()
screen_lock = threading.Lock()

running = True
mqtt_client = None
stdscr = None

def quit(msg):
    global running
    global running_lock
    logging.error(f"Quitting: {msg}")
    with running_lock:
        running = False

def stopped():
    global running
    global running_lock
    with running_lock:
        return not running
    
def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('192.168.1.254', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
        quit("Failed to get local IP")
    finally:
        s.close()
    return ip

# MQTT setup
def on_connect(client,  userdata, flags, rc):
    global mqtt_ready
    global mqtt_lock
    logging.debug(f"Connected with result code {rc}")
    if rc != 0:
        global mqtt_server, mqtt_port
        if rc == 1:
            msg = "Connection refused - incorrect protocol version"
        elif rc == 2:
            msg = "Connection refused - invalid client identifier"
        elif rc == 3:
            msg = "Connection refused - server unavailable"
        elif rc == 4:
            msg = "Connection refused - bad username or password"
        elif rc == 5:
            msg = "Connection refused - not authorized"
        else:
            msg = f"Connection refused - {rc}"
        quit(f"MQTT connect to {mqtt_server}:{mqtt_port} failed - {msg}")
        raise RuntimeError('connection failed')
    
    client.subscribe("status")
    
    logging.debug('connected to mqtt broker')
    with mqtt_ready:
        mqtt_ready.notify_all()

    # Send a message to the broker to let other clients know we are connected
    message = {
        "remote": local_ip()
    }
    with mqtt_lock:
        client.publish("connect", json.dumps(message), qos=1)
    
def on_message(client, userdata, msg):
    global screen_lock
    global stdscr
    data = json.loads(msg.payload.decode('utf-8'))
    if msg.topic == 'status':
        with screen_lock:
            if stdscr:
                stdscr.addstr(5, 0, 'status: ' + str(data))
                stdscr.refresh()

def mqtt_loop():
    global mqtt_client
    global mqtt_lock

    if stopped():
        return
    with mqtt_lock:
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.connect(mqtt_server, mqtt_port, mqtt_keepalive)

    while not stopped():
        mqtt_client.loop(timeout=1.0)
    
    logging.debug('mqtt loop exiting')
    with mqtt_lock:
        mqtt_client.disconnect()
        mqtt_client = None


# Curses Setup
def curses_loop():
    global stdscr
    global screen_lock
    global mqtt_ready

    if stopped():
        return
        
    stdscr = curses.initscr()
    curses.curs_set(0)
    stdscr.nodelay(1)

    height, _ = stdscr.getmaxyx()
    stdscr.addstr(height - 1, 0, "Press 'q' to quit")
    stdscr.refresh()

    while not stopped():
        key = stdscr.getch()
        if key == ord('q'):
            quit("User quit - curses")
        curses.napms(10)    

    logging.debug('curses loop exiting')
    curses.endwin()
    curses.curs_set(1)  # Enable cursor
    stdscr = None


def print_gamepad_data(data):
    global stdscr
    global screen_lock
    with screen_lock:
        if not stdscr:
            return
        stdscr.addstr(0, 0, 'J0     : ' + str(["{:+.2f}".format(i) for i in data["axes"][0:3]]))
        stdscr.addstr(1, 0, 'J1     : ' + str(["{:+.2f}".format(i) for i in data["axes"][3:6]]))
        stdscr.addstr(2, 0, 'HAT    : ' + str(data["hat"]))
        button_str = f"{data['buttons']:011b}"
        stdscr.addstr(3, 0, "Buttons: " + button_str)
        stdscr.refresh()

# Pygame setup
def pygame_loop():
    global mqtt_lock
    global mqtt_client

    if stopped():
        return
    
    pygame.init()
    gamepad = GamepadController()
    last_gamepad = None
    while not stopped():
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYUP and event.key == pygame.K_q):
                quit("User quit - pygame")

        msg = gamepad.get_state()
        if msg != last_gamepad:
            print_gamepad_data(msg)
            last_gamepad = msg
            with mqtt_lock:
                if mqtt_client:
                    mqtt_client.publish("action", json.dumps(msg), qos=1)
        pygame.time.delay(10)
    
    logging.debug('pygame loop exiting')
    pygame.quit()

def main():
    global mqtt_ready
    mqtt_thread = threading.Thread(target=mqtt_loop)
    curses_thread = threading.Thread(target=curses_loop)
    pygame_thread = threading.Thread(target=pygame_loop)
    

    mqtt_thread.start()
    with mqtt_ready:    
        if not mqtt_ready.wait(timeout=10):
            quit("MQTT not ready")
            return

    curses_thread.start()
    pygame_thread.start()
    
    pygame_thread.join()
    curses_thread.join()
    mqtt_thread.join()

if __name__ == "__main__":
    main()
