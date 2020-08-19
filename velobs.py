# pseudo code: simulate basic velocity obstacle for 2 robots only

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

# hacky direct version
def detect(robot1, robot2):
    drel = robot1.pos - robot2.pos
    vrel = polar2cart(robot1.vel, robot1.head) - polar2cart(robot2.vel, robot2.head)
    norm_vrel = np.linalg.norm(vrel)
    if norm_vrel < 0.1:
        norm_vrel = 0.1
    tcpa = - np.inner(drel, vrel) / (norm_vrel**2)
    dcpa = (abs((np.linalg.norm(drel)**2) - ((tcpa ** 2) * (np.linalg.norm(vrel)**2)))) ** (1./2)

    angleb = np.arctan2(robot2.pos[1], robot2.pos[0])
    # do atan dist here
    deltad = np.linalg.norm(robot1.pos - robot2.pos)
    angleb1 = angleb - np.arctan(RR/deltad)
    angleb2 = angleb + np.arctan(RR/deltad)

    centre = robot1.pos + polar2cart(robot2.vel, robot2.head)
    pt1 = centre
    pt2 = centre + np.array([(MAX_VEL * np.cos(angleb1)), (MAX_VEL * np.sin(angleb1))])
    pt3 = centre + np.array([(MAX_VEL * np.cos(angleb2)), (MAX_VEL * np.sin(angleb2))])
    plt.plot([pt2[0], pt1[0], pt3[0]], [pt2[1], pt1[1], pt3[1]], '-o', color='gray', alpha=0.2)
    
    oldvel = robot1.pos + polar2cart(robot1.vel, robot1.head)
    plt.plot(oldvel[0], oldvel[1], 'or', alpha=0.2)
    
    print(tcpa, dcpa)
    if (tcpa > 0) and ((dcpa) < RR):
        newvel = resolve(robot1, robot2)
        avel, robot1.head = cart2polar(newvel)
        robot1.vel = np.clip(avel, -3, 3)
        # print (robot_a.vel, np.rad2deg(robot_a.head))
    else: 
        robot1.vel = robot1.oldvel
        robot1.head = robot1.oldhead
#     print(round(tcpa, 2), round(dcpa, 2))


def project(robot_a, angle2, angle1, centre):
    
    #  body frame to world frame velocity
    vela = robot_a.pos + polar2cart(robot_a.vel, robot_a.head)

    yintercept = centre[1] - (np.tan(angle1) * centre[0])
    # if slope is zero, projection is the x-coordinate itself
    if np.tan(angle2) == 0:
        newvela = np.array([vela[0], 0])
        return newvela
    else:
        xintercept = - yintercept / np.tan(angle1)

    # slope point to slope intercept
    # x_intercept = 3, y intercept = 1.5, slope = -0.5
    # line_prop = [3, 1.5, -0.5]
    line_prop = [xintercept, yintercept, np.tan(angle1)]

    # # slope - intercept form, poly1d(m, c)
    # line1 = np.poly1d([line_prop[2], line_prop[1]])

    # # plot the line
    # x_ax = np.linspace(-10, 10, 10)
    # y_ax = line1(x_ax)
    # plt.plot(x_ax, y_ax)

    # slope - intercept form, poly1d(m, c)
    line1 = np.poly1d([line_prop[2], line_prop[1]])

    # calculate vector passing through origin, y = mx
    vector_from_line = np.array([line_prop[0], -line_prop[1]])
    if (np.inner(vector_from_line, vector_from_line) > 0.1):
        # calculate projection matrix
        # projection matrix P**2 = P
        P = np.outer(vector_from_line, vector_from_line)/np.inner(vector_from_line, vector_from_line)
    else: 
        # back to body frame velocity
        newvela = vela - robot_a.pos
        return newvela
    
    projected_pt = P.dot(vela - np.array([0, line_prop[1]])) + np.array([0, line_prop[1]])
    newvela = projected_pt
    plt.plot(newvela[0], newvela[1], 'og', alpha=0.2)

    # back to body frame velocities
    newvela = newvela - robot_a.pos

    return newvela

def resolve(robot1, robot2):
    # build solution space diagram
    # find angle of b in world frame
    angleb = np.arctan2(robot2.pos[1], robot2.pos[0])
    # do atan dist here
    deltad = np.linalg.norm(robot1.pos - robot2.pos)
    angleb1 = angleb - np.arctan(RR/deltad)
    angleb2 = angleb + np.arctan(RR/deltad)
    # centre of collision cone

    centre = robot1.pos + polar2cart(robot2.vel, robot2.head)
    oldvel = robot1.pos + polar2cart(robot1.vel, robot1.head)
    newvel = project(robot1, angleb1, angleb2, centre)   

    return newvel

robot_obj_list = []
def main():
    config_matplotlib()
    gray = "444444"
    fig = plt.figure()
    plt.grid(True, which='major')
    plt.grid(True, which='minor', color=gray, linestyle = '--', linewidth = 0.5)
    plt.minorticks_on()
    plt.axis('equal')
    plt.xlim(-15, 15)
    plt.ylim(-15, 15)

    robot_a = robot(np.array([-3.0, -3.0]), 2, np.deg2rad(45))
    robot_b = robot(np.array([3.0, 3.0]), 2, np.deg2rad(-135))
    
    robot_obj_list.append(robot_a)
    robot_obj_list.append(robot_b)

    cnt = 0
    for i in range(35):
        # plt.cla()
        detect(robot_a, robot_b)
        detect(robot_b, robot_a)
        plt.plot(block = 'False')
        robot_a.move()
        robot_b.move()
        robot_a.draw(plt)
        robot_b.draw(plt)
        plt.plot()
        filename = 'logs/velobs%02d.png' % cnt
        cnt = cnt + 1
        plt.savefig(filename)
        plt.pause(0.1)
        # q = input('keypress to end')
        # if q != 'c':
        #     break
    plt.show()


if __name__ == "__main__":
    main()