import pymunk
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d
import pymunk.constraints

import pygame
from pygame.locals import *

import math

space = pymunk.Space()
b0 = space.static_body

size = w, h = 800, 800
fps = 30
steps = 10

BLACK = (0, 0, 0)
GRAY = (220, 220, 220)
WHITE = (255, 255, 255)

class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(joint)


class PivotJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)


class SlideJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), min=0, max=0, collide=True):
        joint = pymunk.constraints.SlideJoint(b, b2, a, a2, min, max)
        joint.collide_bodies = collide
        space.add(joint)


class GrooveJoint:
    def __init__(self, a, b, groove_a, groove_b, anchor_b):
        joint = pymunk.constraints.GrooveJoint(
            a, b, groove_a, groove_b, anchor_b)
        joint.collide_bodies = False
        space.add(joint)


class DampedRotarySpring:
    def __init__(self, b, b2, angle, stiffness, damping):
        joint = pymunk.constraints.DampedRotarySpring(
            b, b2, angle, stiffness, damping)
        space.add(joint)


class RotaryLimitJoint:
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)


class RatchetJoint:
    def __init__(self, b, b2, phase, ratchet):
        joint = pymunk.constraints.GearJoint(b, b2, phase, ratchet)
        space.add(joint)


class SimpleMotor:
    def __init__(self, b, b2, rate):
        joint = pymunk.constraints.SimpleMotor(b, b2, rate)
        space.add(joint)


class GearJoint:
    def __init__(self, b, b2, phase, ratio):
        joint = pymunk.constraints.GearJoint(b, b2, phase, ratio)
        space.add(joint)


class Segment:
    def __init__(self, p0, v, radius=10):
        self.body = pymunk.Body()
        self.body.position = p0
        shape = pymunk.Segment(self.body, (0, 0), v, radius)
        shape.density = 0.1
        shape.elasticity = 0.5
        shape.filter = pymunk.ShapeFilter(group=1)
        shape.color = (0, 255, 0, 0)
        space.add(self.body, shape)


class Circle:
    def __init__(self, pos, radius=20):
        self.body = pymunk.Body()
        self.body.position = pos
        shape = pymunk.Circle(self.body, radius)
        shape.density = 0.01
        shape.friction = 0.5
        shape.elasticity = 1
        space.add(self.body, shape)


class Box:
    def __init__(self, p0=(0, 0), p1=(w, h), d=4):
        x0, y0 = p0
        x1, y1 = p1
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i in range(4):
            segment = pymunk.Segment(
                space.static_body, pts[i], pts[(i+1) % 4], d)
            segment.elasticity = 1
            segment.friction = 0.5
            space.add(segment)


class Poly:
    def __init__(self, pos, vertices):
        self.body = pymunk.Body(1, 100)
        self.body.position = pos

        shape = pymunk.Poly(self.body, vertices)
        shape.filter = pymunk.ShapeFilter(group=1)
        shape.density = 0.01
        shape.elasticity = 0.5
        shape.color = (255, 0, 0, 0)
        space.add(self.body, shape)


class Rectangle:
    def __init__(self, pos, size=(100, 50), body_static = False):
        if body_static:
            self.body = pymunk.Body(body_type = pymunk.Body.STATIC)
        else:
            self.body = pymunk.Body()
        self.body.position = pos

        shape = pymunk.Poly.create_box(self.body, size)
        shape.density = 0.1
        shape.elasticity = 1
        shape.friction = 1
        space.add(self.body, shape)


class App:
    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(size)
        self.draw_options = DrawOptions(self.screen)
        self.running = True
        self.images = []

    def run(self):
        while self.running:
            for event in pygame.event.get():
                self.do_event(event)

            self.draw()
            self.clock.tick(fps)

            for i in range(steps):
                space.step(1/fps/steps)

        pygame.quit()

    def do_event(self, event):
        
        if event.type == QUIT:
            self.running = False

        if event.type == KEYDOWN:
            if event.key in (K_q, K_ESCAPE):
                self.running = False

            elif event.key == K_p:
                pygame.image.save(self.screen, 'joint.png')
            
            elif event.key == K_RIGHT:
                orig_x, orig_y = self.rect.body.position
                self.rect.body.position = (orig_x + 10, orig_y)
                space.reindex_shapes_for_body(self.rect.body)
            
            elif event.key == K_LEFT:
                orig_x, orig_y = self.rect.body.position
                if orig_x - 10 > 250:
                    self.rect.body.position = (orig_x - 10, orig_y)
                    space.reindex_shapes_for_body(self.rect.body)


    def draw(self):
        self.screen.fill(GRAY)
        space.debug_draw(self.draw_options)
        pygame.display.update()


if __name__ == '__main__':
    Box()
    p1 = Vec2d(300, 400)
    r1 = Rectangle(p1)
    v1 = (-50, 30)
    SlideJoint(r1.body, b0, v1, p1 + v1)
    # SimpleMotor(r1.body, b0, -5)
    p2 = Vec2d(400, 400)
    r2 = Rectangle(p2)
    v2 = (50, 30)
    # SimpleMotor(r2.body, b0, 5)
    SlideJoint(r1.body, r2.body, v2, v1)
    # PivotJoint(r1.body, r2.body, v2, v1)
    # PivotJoint(r1.body, r2.body, (50,-30), (-50, -30))
    # SimpleMotor(r2.body, r1.body, 5)


    # p0 = Vec2d(400, 300)
    # v = Vec2d(50, 0)

    # arm = Segment(p0, v)
    # PivotJoint(b0, arm.body, p0)
    # SimpleMotor(b0, arm.body, 1)

    # arm2 = Segment(p0+v, v)
    # PivotJoint(arm.body, arm2.body, v, (0, 0))
    # DampedRotarySpring(arm.body, arm2.body, 0, 10000000, 10000)

    r = Rectangle( (500, 450), body_static = True)

    # PivotJoint(r2.body, r.body, v2, Vec2d(-50, -25), True)
    SlideJoint(r2.body, r.body, v2, Vec2d(-50, -25))
    # joint = pymunk.DampedSpring(a=r1.body, b=r2.body, 
    #         anchor_a=(50,-30), anchor_b=(-50,-30), 
    #         rest_length=0, stiffness=800, damping=10)
    # space.add(joint)
    
    a = App()
    a.rect = r
    a.run()
