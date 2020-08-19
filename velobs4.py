# pseudo code: simulate basic velocity obstacle for 2 robots only
# in dev, a bit more elegant velobs2

import matplotlib.pyplot as plt
import numpy as np

MAX_VEL = 20
MAX_DIST = 25
RR = 1
MAX_ROBOTS = 3

def config_matplotlib():
    plt.rcParams['figure.figsize'] = (10, 8)
    gray = "444444"
    plt.rcParams['axes.facecolor'] = 'f5f5f5'
    plt.rcParams['axes.edgecolor'] = gray
    plt.rcParams['grid.linestyle'] = '-'
    plt.rcParams['grid.alpha'] = 0.8
    plt.rcParams['grid.color'] = 'white'
    plt.rcParams['grid.linewidth'] = 2
    plt.rcParams['axes.axisbelow'] = True
    plt.rcParams['axes.labelcolor'] = gray
    plt.rcParams['text.color'] = gray
    plt.rcParams['xtick.color'] = gray
    plt.rcParams['ytick.color'] = gray

def cart2polar(vel_vec):
    mag = np.linalg.norm(vel_vec)
    direction = np.arctan2(vel_vec[1], vel_vec[0])
    return mag, direction

# dir = angle in radians wrt +x
def polar2cart(mag, direction):
    x = mag * np.cos(direction)
    y = mag * np.sin(direction)
    cart = np.array([x, y])
    return cart

class robot:
    def __init__(self, pos, vel, head):
        self.pos = pos
        self.vel = vel
        self.head = head
        # create copies for after conflict resolution
        self.oldvel = vel
        self.oldhead = head

    def move(self):
        [delx, dely] = polar2cart(self.vel*0.2, self.head)
        self.pos[0] += delx
        self.pos[1] += dely

    def draw(self, plt):
        dx, dy = polar2cart(self.vel/20, self.head)
        originpt = self.pos[0], self.pos[1]
        # print (originpt)
        # rad = plt.Circle((self.pos[0], self.pos[1]), RR, color='r', alpha = 0.1)
        # ax = plt.gca()
        # ax.add_artist(rad)
        plt.quiver(*originpt, dx, dy, scale = 10)

# return relative matrix
def rel_mat(robot_idx, num_robots):
    relmat = np.zeros((num_robots, num_robots))
    # strange numpy column assignment
    relmat[:, robot_idx] = np.ones(num_robots)
    relmat = relmat - np.identity(num_robots)
    return relmat

# populate position and velocities of all robots in a matrix
def pos_vel_mat(robot_obj_list):
    posmat = np.zeros((len(robot_obj_list), 2))
    velmat = np.zeros((len(robot_obj_list), 2))

    for i in range(len(robot_obj_list)):
        posmat[i,:] = np.array([robot_obj_list[i].pos[0], robot_obj_list[i].pos[1]])
        velmat[i,:] = polar2cart(robot_obj_list[i].vel, robot_obj_list[i].head)

    return posmat, velmat

# break isolation, share amongst the instances :p
def detect2(robot_obj_list):
    num_robots = len(robot_obj_list)

    # build matrix 
    posmat, velmat = pos_vel_mat(robot_obj_list)

    drel_norm = np.zeros((num_robots, num_robots))
    vrel_norm = np.zeros((num_robots, num_robots))
    tcpa = np.zeros((num_robots, num_robots))
    dcpa = np.zeros((num_robots, num_robots))

    i = 0
    relmat = rel_mat(i, num_robots)
    # relative vel and pos with each robot (3 x 2)
    drel = relmat * posmat
    vrel = relmat * velmat
    # print ('-------------------------')
    # print (i)
    # print (drel)
    # print (vrel)
    # print ('-------------------------')
    
    for j in range(num_robots):
        if j!=i:
            norm_vrel = np.linalg.norm(vrel[j,:])
            if norm_vrel < 0.1:
                norm_vrel = 0.1
            tcpa[i,j] = - np.inner(drel[j,:], vrel[j,:]) / (norm_vrel**2)
            dcpa[i,j] = (abs((np.linalg.norm(drel[j,:])**2) - ((tcpa[i,j]**2) * (np.linalg.norm(vrel[j,:])**2)))) ** (1./2)

    print('----------------------')
    print(np.round(tcpa,2))
    print(np.round(dcpa,2))
    print('----------------------')
    # n x n symmetric matrix is now populated

    # def resolve(robot_obj_list, occupancy_map):