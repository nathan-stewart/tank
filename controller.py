#!/usr/bin/env pythnon3
import pygame
import zmq
import time

pygame.init()
screen = pygame.display.set_mode((640, 480)) 
running = True

# Client (e.g., on your laptop)
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://tank:5555")
socket.setsockopt(zmq.RCVTIMEO, 1000)

def send_message(message):
    socket.send_string(message)
    try:
        reply = socket.recv_string()
        print(f"Received reply: {reply}")
    except zmq.error.Again:
        print("Timeout")

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                send_message('Pressed:Up')
            if event.key == pygame.K_DOWN:
                send_message('Pressed:Down')
            if event.key == pygame.K_LEFT:
                send_message('Pressed:Left')
            if event.key == pygame.K_RIGHT:
                send_message('Pressed:Right')
            if event.key == pygame.K_q:
                running = False

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                send_message('Released:Up')
            if event.key == pygame.K_DOWN:
                send_message('Released:Down')
            if event.key == pygame.K_LEFT:
                send_message('Released:Left')
            if event.key == pygame.K_RIGHT:
                send_message('Released:Right')
