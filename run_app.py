import pymunk
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d
import pymunk.constraints
import math

import pygame
from pygame.locals import *

import math

space = pymunk.Space()
space.gravity = (0, 10)
b0 = space.static_body

size = screen_width, screen_height = 1000, 800
fps = 30
steps = 10

BLACK = (0, 0, 0)
GRAY = (220, 220, 220)
WHITE = (255, 255, 255)

BOT_LEFT = 1
BOT_RIGHT = 2
TOP_LEFT = 3
TOP_RIGHT = 4

class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(joint)


class SlideJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), min=0, max=0, collide=True):
        self.joint = pymunk.constraints.SlideJoint(b, b2, a, a2, min, max)
        self.joint.collide_bodies = collide
        self.max_force = 100000
        space.add(self.joint)

    def is_constrained(self):
        return self.joint.max == 0
    
    def switch_constrain(self):
        if self.is_constrained():
            self.joint._set_max(50)
        else:
            self.joint._set_max(0)



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
        shape.elasticity = 0
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
    def __init__(self, p0=(0, 0), p1=(screen_width, screen_height), d=4):
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
    def __init__(self, pos, size=(100, 50), density = 0.0001, body_static = False):
        if body_static:
            self.body = pymunk.Body(body_type = pymunk.Body.STATIC)
        else:
            self.body = pymunk.Body()
        self.body.position = pos
        self.width = size[0]
        self.height = size[1]
        self.forceFlag = 0
        self.counter = 0
        shape = pymunk.Poly.create_box(self.body, size)
        shape.density = density
        shape.elasticity = 0
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
                print(self.rectangles[-2].counter)
                if self.rectangles[-2].forceFlag == 1:
                    self.rectangles[-2].body.force += ( (0, self.rectangles[-2].counter * -50) )
                if self.rectangles[-2].forceFlag == 2:
                    self.rectangles[-2].body.force += ( (0,  (10 - self.rectangles[-2].counter) * 50) )
                space.step(1/fps/steps)

        pygame.quit()

    def corner_coord(self, body, corner):
        center = body.position

        width = 100
        height = 50

        angle = abs(body.angle) + abs(math.atan(height / width))
        print("angle:", (angle * 180) / math.pi)
        hyp = math.sqrt((height / 2) ** 2 + (width / 2) ** 2)
        x_change = hyp * math.cos(angle)
        y_change = hyp * math.sin(angle)

        if corner == BOT_LEFT:
            return center + Vec2d(-x_change, y_change)
        elif corner == BOT_RIGHT:
            return center + Vec2d(x_change, y_change)
        elif corner == TOP_LEFT:
            return center + Vec2d(-x_change, -y_change)
        else:
            return center + Vec2d(x_change, -y_change)

    def dist(self, p1, p2):
        return math.sqrt( (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 )

    def do_event(self, event):
        
        if event.type == QUIT:
            self.running = False

        if event.type == KEYDOWN:
            if event.key in (K_q, K_ESCAPE):
                self.running = False
            
            elif event.key == K_RIGHT:
                self.actuator.body.apply_force_at_local_point( (100000, 0) )
                # orig_x, orig_y = self.actuator.body.position
                # self.actuator.body.position = (orig_x + 10, orig_y)
                # space.reindex_shapes_for_body(self.actuator.body)
            
            elif event.key == K_LEFT:
                self.actuator.body.apply_force_at_local_point( (-100000, 0) )
                # orig_x, orig_y = self.actuator.body.position
                # self.actuator.body.position = (orig_x - 10, orig_y)
                # space.reindex_shapes_for_body(self.actuator.body)
            
            elif event.key == K_b:
                self.rectangles[-2].forceFlag = 0

            elif event.key == K_UP:
                # self.rectangles[-2].body.apply_force_at_local_point( (0, -100000) )
                self.rectangles[-2].forceFlag = 1
                if self.rectangles[-2].counter < 10:
                    self.rectangles[-2].counter += 1
                # orig_x, orig_y = self.rect_up.body.position
                # self.rect_up.body.position = (orig_x, orig_y - 10)
                # space.reindex_shapes_for_body(self.rect_up.body)
            
            elif event.key == K_DOWN:
                # self.rectangles[-2].body.apply_force_at_local_point( (0, 100000) )
                self.rectangles[-2].forceFlag = 2
                if self.rectangles[-2].counter > 0:
                    self.rectangles[-2].counter -= 1
                # orig_x, orig_y = self.rect_up.body.position
                # self.rect_up.body.position = (orig_x, orig_y + 10)
                # space.reindex_shapes_for_body(self.rect_up.body)

        if event.type == pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            print(pos, type(pos))

            joint1, joint2 = self.joints[0]

            joint1.switch_constrain()
            joint2.switch_constrain()

            # for i in range(len(self.rectangles)):
            #     for corner in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
            #         corner = self.corner_coord(self.rectangles[i].body, corner)
            #         if (self.dist(corner, Vec2d(pos[0], pos[1])) < 20):

            #             if (corner[0] < 0):
            #                 if (i <= 0):
            #                     return
            #                 cur_joint = self.joints[i - 1]
            #             else:
            #                 if (i >= len(self.rectangles) - 1):
            #                     return
            #                 cur_joint = self.joints[i]
                        
            #             cur_joint.switch_constrain()
            #             print("switched constrain!")

            #             return


            # left_rect = self.rectangles[0]
            # right_rect = self.rectangles[1]

            # print("\n\n\n")
            # print("left rect:", left_rect.body.position)
            # print("right rect:", right_rect.body.position)
            # print("left rect ang:", (left_rect.body.angle * 180) / math.pi)
            # print("right rect ang:", (right_rect.body.angle * 180) / math.pi)
            
            # print("\n")
            # print("left rect corner BOT_LEFT:", self.corner_coord(left_rect.body, BOT_LEFT))
            # print("left rect corner TOP_LEFT:", self.corner_coord(left_rect.body, TOP_LEFT))
            # print("left rect corner BOT_RIGHT:", self.corner_coord(left_rect.body, BOT_RIGHT))
            # print("left rect corner TOP_RIGHT:", self.corner_coord(left_rect.body, TOP_RIGHT))
            


    def draw(self):
        self.screen.fill(GRAY)
        space.debug_draw(self.draw_options)
        pygame.display.update()

def five_block_state():
    # Box()

    rectangles = []
    joints = []

    actuator_width = 500

    # rectangle_widths = [50, 200, 50, 200]
    # rectangle_widths = [50, 200, 50, 200]
    rectangle_widths = [50, 50, 50, 50, 50, 50]
    rectangle_widths.append(actuator_width)
    rectangle_height = 50

    starting_x = 100
    starting_y = 400

    start_rect_center = Vec2d(starting_x, starting_y)
    start_rect = Rectangle(start_rect_center, size = (rectangle_widths[0], rectangle_height))
    

    start_rect_bot_left = (-start_rect.width // 2, start_rect.height // 2 + 5)
    SlideJoint(start_rect.body, b0, start_rect_bot_left, start_rect_center + start_rect_bot_left)

    rectangles.append(start_rect)

    left_rect = None
    right_rect = None
    left_rect_center = None
    right_rect_center = None


    for i in range(len(rectangle_widths) - 1):
        left_rect = rectangles[-1]
        left_rect_center = left_rect.body.position
        left_rect_right_top = (left_rect.width // 2, left_rect.height // 2 + 5)

        right_rect_center = left_rect_center + (left_rect.width // 2 + rectangle_widths[i + 1] // 2, 0)
        right_rect = Rectangle(right_rect_center, size = (rectangle_widths[i + 1], rectangle_height))
        right_rect_bot_left = Vec2d(-right_rect.width // 2, right_rect.height // 2 + 5)


        valley = Vec2d(0, - rectangle_height - 10)
        joints.append( (SlideJoint(left_rect.body, right_rect.body, a = left_rect_right_top, a2 = right_rect_bot_left), 
                        SlideJoint(left_rect.body, right_rect.body, a = left_rect_right_top + valley, a2 = right_rect_bot_left + valley, min = 0, max = 50))
        )
        rectangles.append(right_rect)

    # PivotJoint(r1.body, r2.body, v2 + valley, v1 + valley, True)



    # r1 = Rectangle((200, 200)) 
    # r1.body.apply_force_at_local_point( (100000, 0) , (0, 0))

    # actuator_width = 200
    # actuator_height = 50

    # actuator_center = right_rect.body.position + (actuator_width // 2 + actuator_width // 2, 0)

    # actuator = Rectangle( actuator_center, size = (200, 50))
    # up_block = Rectangle( (actuator_x + 100, actuator_y - 50), size = (50, 50), body_static = True)
    # down_block = Rectangle( (actuator_x + 100, actuator_y + 50), size = (50, 50), body_static = True)
    # actuator = Rectangle( (500, 460)) 

    # right_rect_top_right = Vec2d(right_rect.width // 2, -right_rect.height // 2 - 5)
    # right_rect_bot_right = Vec2d(right_rect.width // 2, right_rect.height // 2 + 5)

    # actuator_top_left = Vec2d(-actuator.width // 2, -actuator.height // 2 - 5)
    # actuator_bot_left = Vec2d(-actuator.width // 2, actuator.height // 2 + 5)

    # SlideJoint(right_rect.body, actuator.body, v2, Vec2d(-50, -30))
    # SlideJoint(right_rect.body, actuator.body, right_rect_top_right, actuator_top_left, min = 0, max = 0)

    # SlideJoint(right_rect.body, actuator.body, right_rect_bot_right, actuator_bot_left, min = 0, max = 50)


    pushing_rect = Rectangle( (200, 500), size = (50, 100), body_static = True)
    
    actuator = rectangles[-1]

    up_block = Rectangle( actuator.body.position + (50, -(rectangle_height // 2 + 25)), size = (50, 50), body_static = True)
    down_block = Rectangle( actuator.body.position + (50, (rectangle_height // 2 + 25)), size = (50, 50), body_static = True)

    (actuator_left_joint, actuator_right_joint) =  joints[-1]

    actuator_left_joint.switch_constrain()
    actuator_right_joint.switch_constrain()
    
    a = App()
    a.actuator = actuator
    a.rectangles = rectangles
    a.rect_up = pushing_rect
    a.joints = joints
    a.run()

if __name__ == '__main__':
    print("Five Block mode?: ", end = "")
    five_block = True
    if five_block:
        five_block_state()
    else:
        
        Box()

        rectangles = []
        joints = []

        # joints = []
        p1 = Vec2d(300, 400)
        left_rect = Rectangle(p1)
        v1 = (-50, 30)
        SlideJoint(left_rect.body, b0, v1, p1 + v1)
        p2 = Vec2d(400, 400)
        right_rect = Rectangle(p2)

        rectangles.append(left_rect)
        rectangles.append(right_rect)

        v2 = (50, 30)

        valley = Vec2d(0, -60)
        joints.append( (SlideJoint(left_rect.body, right_rect.body, a = v2, a2 = v1), 
                        SlideJoint(left_rect.body, right_rect.body, a = v2 + valley, a2 = v1 + valley, min = 0, max = 50))
        )

        # PivotJoint(r1.body, r2.body, v2 + valley, v1 + valley, True)



        # r1 = Rectangle((200, 200)) 
        # r1.body.apply_force_at_local_point( (100000, 0) , (0, 0))

        actuator_x = left_rect.width + right_rect.width + p1.x
        actuator_y = 460
        actuator = Rectangle( (actuator_x, actuator_y), size = (200, 50))
        up_block = Rectangle( (actuator_x + 50, actuator_y - 50), size = (50, 50), body_static = True)
        down_block = Rectangle( (actuator_x + 50, actuator_y + 50), size = (50, 50), body_static = True)
        # actuator = Rectangle( (500, 460)) 

        # SlideJoint(right_rect.body, actuator.body, v2, Vec2d(-50, -30))
        SlideJoint(right_rect.body, actuator.body, v2, Vec2d(-actuator.width // 2, -actuator.height // 2  - 5))

        # pushing_rect = Rectangle( (350, 500), size = (50, 100), body_static = True)
        pushing_rect = None

        # rectangles.append(left_rect)
        # rectangles.append(right_rect)
        
        a = App()
        a.actuator = actuator
        a.rect_up = pushing_rect
        a.rectangles = rectangles
        a.joints = joints
        a.run()
