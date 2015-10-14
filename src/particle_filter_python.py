#If this works, we should rewrite it in c++!!!!!!!!!!!!!!!!!!!!!

import random
import math

NUMBER_OF_PARTICLES = 1000
VELOCITY_RANDOMNESS = 0
ROTATION_RANDOMNESS = 0


class Particle:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.w = 0
        self.probability = 0
    def update(self, vel, rotvel):
        vel = vel + random.random()*VELOCITY_RANDOMNESS
        self.w = self.w + rotvel + random.random()*ROTATION_RANDOMNESS
        self.x = self.x + math.cos(self.w)
        self.y = self.y + math.sin(self.w)




class Observation:
    def __init__(self):
        #becomes world coordinates when combined with particle coordinates
        self.x = 0
        self.y = 0
        self.w = 0
        self.measured_distance = 0
        self.estimated_distance_left = 0
        self.estimated_distance_forward = 0
        self.estimated_distance_right = 0
    def get_line_from_observation(self):
        farawayx = self.x + 100*math.cos(self.w)
        farawayy = self.y + 100*math.sin(self.w)
        return (self.x,self.y,farawayx,farawayy)


map = []
particles = []
for i in range(NUMBER_OF_PARTICLES):
    particles.append(Particle())

def particle_filter():
    update()
    get_probabilities()
    resample()
    return get_mean()

def update(vel,wvel):
    for particle in particles:
        particle.update(vel,wvel)


def get_probabilities(observations):
    for particle in particles:
        #observation contains actual distance from sensor
        probability = 0.0
        for o in observations:
            o.x += particle.x
            o.y += particle.y
            wincrement = 0.05
            o.w += particle.w - wincrement
            highestBeamProbability = 0.0
            #Look at three "beams"
            for i in range(3):
                closest = 100000.0
                for wall in map:
                    (ix,iy) = get_intersection(o.get_line_from_observation(),wall)
                    dist = math.sqrt((ix-o.x)*(ix-o.x) + (iy-o.y)*(iy-o.y))
                    if dist < closest:
                        closest = dist
                #get probability that this is the correct distance
                prob = asdfasdfasdf #TODO FIX ZIS
                if prob>highestBeamProbability:
                    highestBeamProbability = prob
            probability+=highestBeamProbability











def resample():
    for particle in particles:
        resample that shit

#takes two lines as input and returns the intersection of those lines
def get_intersection((x11,y11,x12,y12),(x21,y21,x22,y22)):
    k1 = (y12-y11)/(x12-x11)
    k2 = (y22-y21)/(x22-x21)
    m1 = y11-k1*x11
    m2 = y22-k2*x22
    x = (m2-m1)/(k1-k2)
    y = x*k1+m1
    return (x,y)

def get_mean():
    x = 0
    y = 0
    w = 0
    for particle in particles:
        x += particle.x
        y += particle.y
        w += particle.w

    return (x/NUMBER_OF_PARTICLES,y/NUMBER_OF_PARTICLES,w/NUMBER_OF_PARTICLES)