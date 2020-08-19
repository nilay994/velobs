# quick plot pprz values, use full plot if heading info

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt

def main():
    data = genfromtxt('drone_data.csv', delimiter=',')
    plt.minorticks_on()
    plt.axis('equal')
    plt.plot(data[:,0], data[:,1], '.b', label='drone1')
    plt.plot(data[:,4], data[:,5], '.r', label='drone2')
    plt.grid('True')
    plt.show()

if __name__ == "__main__":
    main()
