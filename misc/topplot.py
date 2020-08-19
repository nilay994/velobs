# quick plot pprz values, use full plot if heading info

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

def config_matplotlib():
    plt.rcParams['figure.figsize'] = (10, 10)
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


def main():
    data = genfromtxt('drone_data.csv', delimiter=',')
    config_matplotlib()
    plt.minorticks_on()
    # plt.axis('equal')
    ax1 = plt.subplot(2,2,1)
    ax1.plot(data[:, 8], data[:, 3] * 180.0/3.142)
    ax2 = plt.subplot(2,2,2)
    ax2.plot(data[:, 8], data[:, 2])

    ax3 = plt.subplot(2,2,3)
    ax3.legend(['drone1', 'drone2'])
    
    for i in range(np.size(data,0)):
        ax3.plot(data[i,0], data[i,1], '.b', label='drone1')
        ax3.plot(data[i,4], data[i,5], '.r', label='drone2')

        # for MAG
        # plt.plot(data[i, 8], np.rad2deg(data[i, 3]), '-r')

        print(np.round([data[i, 1], data[i, 0], data[i, 2], np.rad2deg(data[i, 3])], 3), end =" ")
        print("|", end =" ")
        print(np.round([data[i, 5], data[i, 4], data[i, 6], np.rad2deg(data[i, 7])], 3))
        plt.xlim([-5, 5])
        plt.ylim([-5, 5])
        plt.grid('True')
        plt.pause(0.01)
    
    plt.show()

if __name__ == "__main__":
    main()
