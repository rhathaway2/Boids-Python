from p5 import circle, line, setup, draw, size, run, background, stroke, fill
import numpy as np
import math
import random


'''
Constants
'''
VIEW_DISPLAY_ON = False

WIDTH = 1500
HEIGHT = 900

BOID_COUNT = 35

VIEWING_DIST = 100.0
VIEWING_ANGLE = 80.0

MAX_SPEED = 12.0
MIN_SPEED = 6.0

WALL_FACTOR = 0.45
WALL_FORCE = 5.0
WALL_DISTANCE = 35.0

MAX_COHESION_FORCE= 5.0
MAX_SEPERATION_FORCE=5.0
SEPERATION_DIST = 40.0

#Factors for different rule strengths
ALIGNMENT_FACTOR = 0.35
COHESION_FACTOR = 0.10
SEPERATION_FACTOR = 0.4

'''
magnitude: gets magnitude from vector x and y
'''
def magnitude(x,y):
    return math.sqrt((x**2)+(y**2))

'''
Limit magnitude function

limites magnitude of a given vector to a specified range
'''
def limit_magnitude(vector, max_mag, min_mag=0.0):
    mag = magnitude(*vector)
    if mag == 0:
        return vector
    elif mag > max_mag:
        normalizing_factor = max_mag / mag
    elif mag < min_mag:
        normalizing_factor = min_mag / mag
    else:
        return vector
    
    return [value * normalizing_factor for value in vector]



'''
Boid class

represents individuals Boids in simulation
'''
class Boid:
    '''
    Init function: initializes a boid
    arguments
    '''
    def __init__(self, id=-1, position=[500.0, 500.0], velocity=[0.0, 0.0], acceleration=[1.0, 1.0]):
        self.id = id
        self.pos = np.asarray(position)
        self.vel = np.asarray(velocity)
        self.accel = np.asarray(acceleration)

    '''
    show function: draws boid on screen
    '''
    def show(self, boid_list):
        y_angle = math.atan(self.vel[1] / self.vel[0])
        x_angle = math.atan(self.vel[0] / self.vel[1])
        larger = x_angle if x_angle > y_angle else y_angle
        lsign = -1 if larger < 0 else 1

        norm_x = x_angle / (10.0 * lsign)
        norm_y = y_angle / (10.0 * lsign)

        x_mult = -1 if self.vel[0] < 0 else 1
        y_mult = -1 if self.vel[1] < 0 else 1

        x_point_coord = self.pos[0] + (x_mult * norm_x * 150)
        y_point_coord = self.pos[1] + (y_mult * norm_y * 150)

        if self.id == 0 and VIEW_DISPLAY_ON:
            stroke("gray", alpha=100.5, v2=80)
            fill("gray", alpha=100.5, v2=80)
            circle((self.pos), radius=2*VIEWING_DIST)

            stroke("blue")
            for boid in boid_list:
                x_coord_difference = self.pos[0] - boid.pos[0]
                y_coord_difference = self.pos[1] - boid.pos[1]
                dist = magnitude(x_coord_difference, y_coord_difference)
                #if within min distance
                if dist <= VIEWING_DIST:
                    try:
                        line( self.pos, boid.pos )
                    except:
                        pass

        #get random color
        stroke(255)
        fill(255)
        circle(self.pos, radius=10)
        stroke("red")
        line(self.pos, (x_point_coord, y_point_coord))


    '''
    Find nearby boids
    '''
    def find_nearby_boids(self):
        nearby_boids = []
        for boid in boid_list:
            #get distance between boids
            x_coord_difference = self.pos[0] - boid.pos[0]
            y_coord_difference = self.pos[1] - boid.pos[1]
            dist = magnitude(x_coord_difference, y_coord_difference)
            #if within min distance
            if dist <= VIEWING_DIST:
                nearby_boids.append(boid)
        return nearby_boids
        
        

    '''
    avoid obstacles
    '''
    def avoid_obstacles(self):
        pass

    '''
    movement_noise
    '''
    def movement_noise(self):
        pass

    '''
    allignment: moves boid towards average velocity of nearby boids
    arguments:
    boid_list: a list of other boids
    '''
    def allignment(self, boid_list):
        total=0
        avg_vel = np.asarray([0.0,0.0])
        #get average velocity of nearby boids
        for boid in boid_list:
            #get distance between boids
            x_coord_difference = self.pos[0] - boid.pos[0]
            y_coord_difference = self.pos[1] - boid.pos[1]
            dist = magnitude(x_coord_difference, y_coord_difference)
            #if within min distance
            if dist <= VIEWING_DIST:
                total += 1
                avg_vel += boid.vel
        if total > 0:
            avg_vel = avg_vel / float(total)
            #update acceleration
            self.accel = (self.accel + (avg_vel - self.vel)) * ALIGNMENT_FACTOR

    '''
    cohesion
    keep boids generally close to each other
    '''
    def cohesion(self, boid_list):
        total = 0
        avg_pos= np.asarray([0.0, 0.0])
        for boid in boid_list:
            #get distance between boids
            x_coord_difference = self.pos[0] - boid.pos[0]
            y_coord_difference = self.pos[1] - boid.pos[1]
            dist = magnitude(x_coord_difference, y_coord_difference)
            #if within min distance
            if dist <= VIEWING_DIST:
                total += 1
                avg_pos += boid.pos
        if total > 0:
            avg_pos = avg_pos / total
            avg_pos = avg_pos - self.pos
            if np.linalg.norm(avg_pos) > 0:
                avg_pos = (avg_pos / np.linalg.norm(avg_pos)) * MAX_SPEED

            avg_pos = avg_pos - self.vel
            if np.linalg.norm(avg_pos) > MAX_COHESION_FORCE:
                avg_pos = (avg_pos / np.linalg.norm(avg_pos)) * MAX_COHESION_FORCE
            
            #update acceleration
            self.accel = (self.accel + avg_pos * COHESION_FACTOR)


    '''
    seperation
    keep boids from getting too close to one another
    '''
    def seperation(self, boid_list):
        total = 0
        avg_pos = np.asarray([0.0, 0.0])
        for boid in boid_list:
            #get distance between boids
            x_coord_difference = self.pos[0] - boid.pos[0]
            y_coord_difference = self.pos[1] - boid.pos[1]
            dist = magnitude(x_coord_difference, y_coord_difference)
            #if within min distance and boid is not self
            if dist <= SEPERATION_DIST and (x_coord_difference != 0 or y_coord_difference != 0):
                diff = self.pos - boid.pos
                diff = diff / dist
                total += 1
                avg_pos += diff
        if total > 0:
            avg_pos = avg_pos / total
            if np.linalg.norm(avg_pos) > 0:
                avg_pos = (avg_pos / np.linalg.norm(avg_pos)) * MAX_SPEED

            avg_pos = avg_pos - self.vel
            if np.linalg.norm(avg_pos) > MAX_SEPERATION_FORCE:
                avg_pos = (avg_pos / np.linalg.norm(avg_pos)) * MAX_SEPERATION_FORCE
            
            #update acceleration
            self.accel = (self.accel + avg_pos * SEPERATION_FACTOR)

    '''
    avoid walls function
    '''
    def avoid_walls(self):
        if self.pos[0] >= WIDTH - WALL_DISTANCE:
            self.vel[0] = self.vel[0] - WALL_FORCE * WALL_FACTOR
        elif self.pos[0] <= WALL_DISTANCE:
            self.vel[0] = self.vel[0] + WALL_FORCE * WALL_FACTOR
        if self.pos[1] >= HEIGHT - WALL_DISTANCE:
            self.vel[1] = self.vel[1] - WALL_FORCE * WALL_FACTOR
        elif self.pos[1] <= WALL_DISTANCE:
            self.vel[1] = self.vel[1] + WALL_FORCE * WALL_FACTOR

    '''
    move function: moves boid based on velocity and acceleration
    '''
    def move(self):
        #update position of boid
        self.pos = np.sum([self.pos, self.vel], axis=0)
        self.vel = np.sum([self.vel, self.accel], axis=0)

        #acceleration limit
        if np.linalg.norm(self.vel) > MAX_SPEED:
            self.vel = self.vel / np.linalg.norm(self.vel) * MAX_SPEED

        #upper and lower bound for velocity
        self.vel = limit_magnitude(self.vel, MAX_SPEED, MIN_SPEED)

        self.avoid_walls()

        #reset acceleration to 0
        self.accel = np.asarray([0.0,0.0])


#boids
boid_list = [ Boid( id=i, 
                    position=[20+random.random()*WIDTH, 20+random.random()*HEIGHT],
                    velocity=[random.choice([-3,3]), 
                    acceleration=random.choice([-3,3])]) for i in range(BOID_COUNT)
            ]



'''
Boids Simulation Display

'''
def setup():
    size(WIDTH, HEIGHT)


def draw():
    background(30, 30, 47)
    for boid in boid_list:
        nearby_boids = boid.find_nearby_boids()
        boid.show(nearby_boids)
        boid.allignment(nearby_boids)
        boid.cohesion(nearby_boids)
        boid.seperation(nearby_boids)
        boid.move()


#run simulation
run(frame_rate=60)
