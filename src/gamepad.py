import pygame
import logging

class GamepadController:
    def __init__(self):
        self.pygame = pygame
        self.pub = None

    def init_gamepad(self):
        self.pygame.init()
        self.pygame.joystick.init()
        if self.pygame.joystick.get_count() > 0:
            self.joystick = self.pygame.joystick.Joystick(0)
            self.joystick.init()
            logging.log(logging.INFO, "Gamepad initialized.")
        else:
            logging.log(logging.FATAL, "No Gamepad detected.")

    def read_input(self):
        self.pygame.event.pump()
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        hat  = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
        buttons = 0;
        for i in range(self.joystick.get_numbuttons()):
            buttons += self.joystick.get_button(i) << i
        return axes, hat, buttons
    
    def rumble(self, shake):
        self.joystick.rumble(0.8, 0.8, 1000)
