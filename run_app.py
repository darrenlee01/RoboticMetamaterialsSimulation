import pygame
import pymunk
import pymunk.pygame_util
import math

pygame.init()

def draw(space, window, draw_options):
    window.fill("white")
    space.debug_draw(draw_options)
    pygame.display.update()

width, height = 800, 800
window = pygame.display.set_mode((width, height))

def run (window, width, height):
    run = True
    clock = pygame.time.Clock()
    fps = 60

    dt = 1 / fps

    space = pymunk.Space()
    space.gravity = (0, 981)

    draw_options = pymunk.pygame_util.DrawOptions(window)

    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break
        
        draw(space, window, draw_options)
        space.step(dt)
        clock.tick(fps)
    pygame.quit()

if __name__ == "__main__":
    run(window, width, height)