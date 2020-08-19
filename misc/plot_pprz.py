# plot pprz values

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

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
        print(self.pos)
        self.firstRun = True

    def draw(self, currpos, currvel, currhead, plt):
        self.pos = currpos
        self.vel = currvel
        self.head = currhead
        dx, dy = polar2cart(self.vel/2, self.head)
        originpt = self.pos[0], self.pos[1]
        # global firstRun
        global qv

        if (self.firstRun):
            qv = plt.quiver(*originpt, dx, dy, scale = 10)
            qv.set_color('black')
            qv.set_alpha(0.9)
            self.firstRun=False
            print("first.......")
        
        if (self.firstRun==False):
            qv.set_color('black')
            qv.set_alpha(1.0)
            qv = plt.quiver(*originpt, dx, dy, scale = 10)
            qv.set_color('black')
            qv.set_alpha(1.0)

        # print (originpt)
        # rad = plt.Circle((self.pos[0], self.pos[1]), RR, color='r', alpha = 0.1)
        # ax = plt.gca()
        # ax.add_artist(rad)

def main():
    data = genfromtxt('drone_data.csv', delimiter=',')
    print(data[0, 0:1])
    robota = robot(data[0, [0,1]], data[0, 2], data[0, 3])
    robotb = robot(data[0, [4,5]], data[0, 6], data[0, 7])
    
    plt.minorticks_on()
    plt.axis('equal')
    plt.xlim(-30, 30)
    plt.ylim(-30, 30)

    for i in range (np.size(data, 0)):
        plt.plot(block = 'False')
        plt.grid('True')
        robota.draw(data[i, [0,1]], data[i, 2], data[i, 3], plt)
        robotb.draw(data[i, [4,5]], data[i, 6], data[i, 7], plt)
        print(np.round([data[i, 0], data[i, 1], data[i, 2], data[i, 3]], 3), end =" ")
        print("|", end =" ")
        print(np.round([data[i, 4], data[i, 5], data[i, 6], data[i, 7]], 3))
        plt.pause(0.1)
    plt.show()

if __name__ == "__main__":
    main()