import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import Box2D  # The main library
import math
import numpy as np

class CircleBody():
    def __init__(self, world: Box2D.b2World, radius: float, pos: tuple[float], mass=1.0, friction=1.0, restitution=0.0):
        body: Box2D.Box2D.b2Body = world.CreateDynamicBody(position=pos)
        density = mass / (math.pi * radius**2)
        self.fixture: Box2D.Box2D.b2Fixture = body.CreateCircleFixture(radius=radius, density=density, friction=friction)
        
        # ~~~~~~~~~~~~~~~~~~~~
        self.circle_surface = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(self.circle_surface, (255, 0, 0), (radius, radius), radius)
        pygame.draw.circle(self.circle_surface, (0, 0, 255), (radius + 30, radius), 15)
    
    @property
    def position(self):
        return self.fixture.body.position
    
    @property
    def shape(self):
        return self.fixture.shape
    

class PIDController():
    def __init__(self, P=1, I=0.1, D=0.2):
        self.P = P
        self.I = I
        self.D = D
        self.integral = 0
        self.previous = 0
        
    def getOmega(self, theta):
        self.I += theta
        derivative = theta -self.previous
        self.previous = theta
        
        return self.P * theta + self.I * self.integral + self.D * derivative
        

class Simulation():
    # --- constants ---
    # Box2D deals with meters, but we want to display pixels,
    # so define a conversion factor:
    PPM = 800.0  # pixels per meter
    TARGET_FPS = 60
    TIME_STEP = 1.0 / TARGET_FPS
    SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480
    def __init__(self):
        # --- pygame setup ---
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), 0, 32)
        pygame.display.set_caption('Simple pygame example')
        self.clock = pygame.time.Clock()
        # --- pybox2d world setup ---
        # Create the world
        self.world = Box2D.b2World(gravity=(0, -9.81), doSleep=True)
        # Create the ground
        self.groundBody = self.world.CreateStaticBody(position=(0, 0), shapes=Box2D.b2PolygonShape(box=(50, 0.1)))
        self.groundBody.fixtures[0].friction = 1.0
        
        # Create the ball
        self.ball = CircleBody(self.world, radius=0.115, pos=(0.4, 0.22), mass=3.2)
        self.robot = CircleBody(self.world, 0.05, (0.41, 0.4), mass=7.135)
        
        self.colors = {
            
            Box2D.b2_staticBody: (255, 255, 255, 255),
            Box2D.b2_dynamicBody: (127, 127, 127, 255),
        }
        
        # --- pybox2d sim setup ---
        def my_draw_circle(circle, body, fixture):
            rotated_surface = pygame.transform.rotozoom(self.ball.circle_surface, body.angle, 1)
            rotated_rect = rotated_surface.get_rect(center=body.transform * circle.pos * self.PPM)

            # Blit the rotated surface onto the screen
            self.screen.blit(rotated_surface, rotated_rect.topleft)
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # position = body.transform * circle.pos * self.PPM
            # position = (position[0], self.SCREEN_HEIGHT - position[1])
            # pygame.draw.circle(self.screen, self.colors[body.type], [int(
            #     x) for x in position], int(circle.radius * self.PPM))
        Box2D.b2CircleShape.draw = my_draw_circle
        
        def my_draw_polygon(polygon, body, fixture):
            vertices = [(body.transform * v) * self.PPM for v in polygon.vertices]
            vertices = [(v[0], self.SCREEN_HEIGHT - v[1]) for v in vertices]
            pygame.draw.polygon(self.screen, self.colors[body.type], vertices)
        Box2D.b2PolygonShape.draw = my_draw_polygon
        
    def setSpeed(self, speed):
        # pps = speed / 0.9
        # if pps <= 300: torque = 160 / 100
        # elif pps >= 4700: torque = 5 / 100
        # else: torque = (1E-19 * pps**6 - 2E-15 * pps**5 + 1E-11 * pps**4 - 2E-08 * pps**3 + 4E-06 * pps**2 - 0.0385 * pps + 172.2) / 100
        # self.robot.fixture.body.ApplyTorque(torque, wake=True)
        self.robot.fixture.body.angularVelocity = speed
        
    def sim(self, controlFunction):
        # --- main game loop ---
        running = True
        while running:
            # Check the event queue
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    # The user closed the window or pressed escape
                    running = False
                    
            self.screen.fill((0, 0, 0, 0))
            # Draw the world
            for body in self.world.bodies:  # or: world.bodies
                # The body gives us the position and angle of its shapes
                for fixture in body.fixtures:
                    # The fixture holds information like density and friction,
                    # and also the shape.
                    fixture.shape.draw(body, fixture)

            self.setSpeed(controlFunction(self.getTheta()))
            self.world.Step(self.TIME_STEP, 10, 10)
            # Flip the screen and try to keep at the target FPS
            pygame.display.flip()
            self.clock.tick(self.TARGET_FPS)
            
        pygame.quit()
        
    def getTheta(self):
        vec = self.robot.position - self.ball.position
        costheta = vec.dot(Box2D.b2Vec2(0,1)) / vec.length
        return math.acos(costheta)

def getTorque(x):
    y = 1E-19 * x**6 - 2E-15 * x**5 + 1E-11 * x**4 - 2E-08 * x**3 + 4E-06 * x**2 - 0.0385 * x + 172.2

        
sim = Simulation()
controller = PIDController(P=244, I=0, D=0)
sim.sim(controller.getOmega)