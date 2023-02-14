import pygame
import pymunk
import pymunk.pygame_util
import math

pygame.init()
space = pymunk.Space()

width, height = 800, 800
window = pygame.display.set_mode((width, height))

class RotaryLimitJoint:
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraint.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)

def run (window, width, height):
    run = True
    fps = 60
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break
    pygame.quit()