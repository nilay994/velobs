# simulation preudo code: simulate basic velocity obstacle for 2 robots only
# use the dcpa tcpa analogy to predict whether any collision is possible (Dennis)
# red dot: original velocity, green dot: resolved velocity

import matplotlib.pyplot as plt
import numpy as np

VEL_CONE = 20.0
MAX_VEL = 3.0
RR = 10.0

def config_matplotlib():
    plt.rcParams['figure.figsize'] = (10, 10)
    gray = '444444'
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
    def __init__(self, pos, vel, initvel):
        self.pos = pos
        self.vel = vel
        # create copies for after conflict resolution
        self.initvel = initvel
        print("-------------Initvel----------")
        print(initvel)
        self.firstRun = True

    def move(self):
        # [TODO] to get sizeable plots, scale vel
        self.pos[0] += self.vel[0]
        self.pos[1] += self.vel[1]

    def draw(self, plt):
        dx = 0.05 * self.vel[0]
        dy = 0.05 * self.vel[1]
        originpt = self.pos[0], self.pos[1]
        # global firstRun
        global qv

        if (self.firstRun):
            qv = plt.quiver(*originpt, dx, dy, scale = 10)
            qv.set_color('black')
            qv.set_alpha(1.0)
            self.firstRun=False
        
        if (self.firstRun==False):
            qv.set_color('black')
            qv.set_alpha(0.2)
            qv = plt.quiver(*originpt, dx, dy, scale = 10)
            qv.set_color('black')
            qv.set_alpha(1.0)

        # draw RR
        # print (originpt)
        # rad = plt.Circle((self.pos[0], self.pos[1]), RR, color='r', alpha = 0.1)
        # ax = plt.gca()
        # ax.add_artist(rad)

cnttrue = 0
cntfalse = 0
# hacky direct version
def detect(robot1, robot2):
    global cnttrue, cntfalse
    drel = robot1.pos - robot2.pos
    vrel = robot1.vel - robot2.vel

    norm_drel = np.linalg.norm(drel)    
    norm_vrel = np.linalg.norm(vrel)

    if norm_vrel < 0.1:
        norm_vrel = 0.1
    # collision time elapsed, negative value infers collision was expected in the past
    tcpa = - np.inner(drel, vrel) / (norm_vrel**2)
    # distance between bots when collision is predicted
    dcpa = (abs((norm_drel**2) - ((tcpa ** 2) * (norm_vrel**2)))) ** (1.0/2)

    # print(round(tcpa, 2), round(dcpa, 2))
    angleb = np.arctan2(robot2.pos[1]-robot1.pos[1], robot2.pos[0]-robot1.pos[0])
    deltad = norm_drel
    angleb1 = angleb - np.arctan(RR/deltad)
    angleb2 = angleb + np.arctan(RR/deltad)

    centre = robot1.pos + robot2.vel
    pt1 = centre
    pt2 = centre + np.array([(VEL_CONE * np.cos(angleb1)), (VEL_CONE * np.sin(angleb1))])
    pt3 = centre + np.array([(VEL_CONE * np.cos(angleb2)), (VEL_CONE * np.sin(angleb2))])
    plt.plot([pt2[0], pt1[0], pt3[0]], [pt2[1], pt1[1], pt3[1]], '-o', color='gray', alpha=0.2)
    
    oldvel = robot1.pos + robot1.vel
    plt.plot(oldvel[0], oldvel[1], 'or', alpha=0.2)

    # too close, don't take decisions
    if (norm_drel < 1.25 * RR):
        robot1.vel = robot1.initvel
        return

    # tcpa > 0: collision in future
    # dcpa < RR: distance during collision
    # tcpa < 6: ignore distant future, evade if crash expected in a few (6) seconds 
    # deltad > RR condition avoids planning when collision cone is too wide
    if ((tcpa > 0) and (tcpa < 20) and (dcpa < RR)):
        cnttrue = cnttrue + 1
        cntfalse = 0
    if ((tcpa < -0.5)):
        cnttrue = 0
        cntfalse = cntfalse + 1
    
    # print(cnttrue, cntfalse)
    # making-sure filter
    if (cnttrue > 5):
        obs_phase = np.arctan2(robot2.vel[1], robot2.vel[0])
        # force right: for solving co-ordination problem
        # todo: adopt vector dot product from pprz branch instead of atan2 compare
        if (np.abs(obs_phase - angleb2) < (np.abs(obs_phase - angleb1) + 0.2)):
            newvel = vo_resolve_by_project(robot1, angleb1, angleb2, centre)
            print("should resolve on b1: lower angle: " + str(angleb1*180.0/3.142))
        else:
            newvel = vo_resolve_by_project(robot1, angleb2, angleb1, centre)
            print("should resolve on b2: higher angle: " + str(angleb2*180.0/3.142))
        
        robot1.vel[0] = np.clip(newvel[0], -MAX_VEL, MAX_VEL)
        robot1.vel[1] = np.clip(newvel[1], -MAX_VEL, MAX_VEL)
    
    if (cntfalse > 5):
        # TODO: WHAT!: How is initvel changing? It is only done in the constructor!!
        print("Init vel later!!:::!!!!werewrwerwer3242342342")
        print(robot1.initvel, id(robot1.initvel))
        robot1.vel = robot1.initvel

def vo_resolve_by_project(robot_a, angle1, angle2, centre):
    
    # body frame to world frame velocity
    vela = robot_a.pos + robot_a.vel

    yintercept = centre[1] - (np.tan(angle1) * centre[0])
    # if slope is zero, projection is the x-coordinate itself
    if np.tan(angle2) == 0:
        newvela = np.array([vela[0], 0]) - robot_a.pos
        return newvela
    else:
        xintercept = - yintercept / np.tan(angle1)
        
    line_prop = [xintercept, yintercept, np.tan(angle1)]

    # slope - intercept form, poly1d(m, c)
    line1 = np.poly1d([line_prop[2], line_prop[1]])

    # calculate vector passing through origin, y = mx
    vector_from_line = np.array([line_prop[0], -line_prop[1]])
    if (np.inner(vector_from_line, vector_from_line) > 0.1):
        # calculate projection matrix
        # projection matrix P**2 = P
        P = np.outer(vector_from_line, vector_from_line)/np.inner(vector_from_line, vector_from_line)

        projected_pt = P.dot(vela - np.array([0, line_prop[1]])) + np.array([0, line_prop[1]])
        newvela = projected_pt
        plt.plot(newvela[0], newvela[1], 'og', alpha=0.2)
        # back to body frame velocities
        newvela = newvela - robot_a.pos
        return newvela
    else: 
        # back to body frame velocity
        newvela = vela - robot_a.pos
        return newvela

robot_obj_list = []
def main():
    config_matplotlib()
    fig = plt.figure()
    plt.grid(True, which='major', color='gray', linestyle = '-', linewidth = 1)
    plt.grid(True, which='minor', color='gray', linestyle = '--', linewidth = 0.5)
    plt.minorticks_on()
    plt.axis('equal')
    plt.xlim(-50, 50)
    plt.ylim(-50, 50)

    robot_a = robot(np.array([-40.0, 40.0]), np.array([1.0, -1.0]), np.array([1.0, -1.0]))
    robot_b = robot(np.array([-40.0, -40.0]), np.array([1.0, 1.0]), np.array([1.0, 1.0]))
    
    robot_obj_list.append(robot_a)
    robot_obj_list.append(robot_b)

    cnt = 0
    toggle = 0
    for i in range(70):
        # plt.cla()
        
        plt.plot(block = 'False')
        robot_a.move()
        robot_b.move()

        # add esp32 delay
        # if i % 8 == 0:
        #     robot_b.move()

        robot_a.draw(plt)
        robot_b.draw(plt)

        # robot_a is in avoid mode,
        # if robot_a arg is passed before robot_b
        detect(robot_a, robot_b)

        # if (abs(robot_b.pos[1]) > 7):
        #     toggle = ~toggle
        #     if toggle:
        #         robot_b.head = np.deg2rad(135.0)
        #     else:
        #         robot_b.head = np.deg2rad(-45.0)

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