import pygame
import logging
import json

class GamepadController:
    def __init__(self):
        pygame.joystick.init()
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logging.log(logging.INFO, "Gamepad initialized.")
        else:
            logging.log(logging.FATAL, "No Gamepad detected.")

    def get_state(self):
        pygame.event.pump()
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        hat  = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
        buttons = 0;
        for i in range(self.joystick.get_numbuttons()):
            buttons += self.joystick.get_button(i) << i
            
        message = {
            "axes": axes,
            "hat": hat,
            "buttons": buttons
        }
        return message
    