# TODO If this works, we should rewrite it in c++!!!!!!!!!!!!!!!!!!!!!

import random
import math
import copy

NUMBER_OF_PARTICLES = 1000
VELOCITY_RANDOMNESS = 0
ROTATION_RANDOMNESS = 0
GAUSSIAN_STANDARD_DEVIATION = 0.1


class Particle:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.w = 0
        self.probability = 0

    def update(self, vel, rotvel):
        vel += random.random()*VELOCITY_RANDOMNESS
        self.w += rotvel + random.random()*ROTATION_RANDOMNESS
        self.x += vel*math.cos(self.w)
        self.y += vel*math.sin(self.w)


class Observation:
    def __init__(self):
        # becomes world coordinates when combined with particle coordinates
        self.x = 0
        self.y = 0
        self.w = 0
        self.measured_distance = 0
        self.estimated_distance_left = 0
        self.estimated_distance_forward = 0
        self.estimated_distance_right = 0

    def get_line_from_observation(self):
        farawayx = self.x + 100.0*math.cos(self.w)
        farawayy = self.y + 100.0*math.sin(self.w)
        return self.x, self.y, farawayx, farawayy


def particle_filter(observations, vvel, wvel):
    update(vvel, wvel)
    get_probabilities(observations)
    resample(particles)
    return get_mean()


def update(vel, wvel):
    for particle in particles:
        particle.update(vel, wvel)


def get_probabilities(observations):
    total_probability = 0.0
    for particle in particles:
        # observation contains actual distance from sensor
        probability = 0.0
        for o in observations:

            o.x += particle.x
            o.y += particle.y
            wincrement = 0.05
            o.w += particle.w - wincrement
            highest_beam_probability = 0.0
            # Look at three "beams" TODO could be done with more or less beams
            for i in range(3):

                closest = 100000.0
                # TODO Should only look in a particular direction
                for wall in wall_map:
                    (ix, iy) = get_intersection(o.get_line_from_observation(), wall)
                    dist = math.sqrt((ix-o.x)*(ix-o.x) + (iy-o.y)*(iy-o.y))
                    if dist < closest:
                        closest = dist
                # get probability that this is the correct distance
                # TODO really check if this is correct or not!!!!!!!!!!!!
                prob = math.erf(closest/GAUSSIAN_STANDARD_DEVIATION) # TODO not sure if this is the correct use of the error function!!!!!
                if prob > highest_beam_probability:
                    highest_beam_probability = prob
                o.w += wincrement
            probability += highest_beam_probability
        particle.probability = probability
        total_probability += probability
    # Normalize
    # TODO Could do resample() here as well for speed increase
    for particle in particles:
        particle.probability = particle.probability/total_probability


def resample(particles):
    cumsum = 0.0
    problist = []
    for particle in particles:
        cumsum += particle
        problist.append(cumsum)

    # This is N^2, is this really the best we can do???
    for particle in particles:
        random_number = random.random()
        i = 0
        for prob in problist:
            if random_number > prob:
                particle = copy.copy(particles[i])
                break
            i += 1


# takes two lines as input and returns the intersection of those lines in 2D
def get_intersection((x11, y11, x12, y12), (x21, y21, x22, y22)):
    k1 = (y12-y11)/(x12-x11)
    k2 = (y22-y21)/(x22-x21)
    m1 = y11-k1*x11
    m2 = y22-k2*x22
    x = (m2-m1)/(k1-k2)
    y = x*k1+m1
    return x, y


def get_mean():
    x = 0
    y = 0
    w = 0
    for particle in particles:
        x += particle.x
        y += particle.y
        w += particle.w

    return x/NUMBER_OF_PARTICLES, y/NUMBER_OF_PARTICLES, w/NUMBER_OF_PARTICLES


wall_map = []
particles = []
for counter in range(NUMBER_OF_PARTICLES):
    particles.append(Particle())
