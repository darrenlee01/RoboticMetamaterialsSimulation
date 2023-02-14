import pygame
import pymunk
import pymunk.pygame_util
import math
from joints import b0, DampedRotarySpring, PivotJoint, RotaryLimitJoint, SimpleMotor, Vec2d

pygame.init()
space = pymunk.Space()

width, height = 800, 800
window = pygame.display.set_mode((width, height))



def run (window, width, height):
    run = True
    fps = 60
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break
    pygame.quit()