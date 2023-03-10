import numpy as np
import matplotlib.pylab as plt
import os
from ament_index_python.packages import get_package_share_directory

if __name__ == '__main__':

    share_tmp_dir = os.path.join(get_package_share_directory('mini_project'), 'tmp')
    file_path = os.path.join(share_tmp_dir, 'visual_field_logging.txt')
    data = np.genfromtxt(file_path, delimiter=',')
    print(data)
    plt.figure()
    plt.plot(data[:, 0], data[:, 1], 'b', label='true')

    # Cylinders.
    a = np.linspace(0., 2 * np.pi, 20)
    x = np.cos(a) * .3 + -1.12
    y = np.sin(a) * .3 + .9
    plt.plot(x, y, 'cyan')

    x = np.cos(a) * .3 + .04
    y = np.sin(a) * .3 - 1.09
    plt.plot(x, y, 'g')

    x = np.cos(a) * .3 - .58
    y = np.sin(a) * .3 - .19
    plt.plot(x, y, 'y')

    x = np.cos(a) * .3 + .42
    y = np.sin(a) * .3 + .79
    plt.plot(x, y, 'b')

    #Ball.
    x = np.cos(a) * .26 - 1.64
    y = np.sin(a) * .26 + 1.62
    plt.plot(x, y, 'r')

    # Walls.
    plt.plot([-2, 2], [-2, -2], 'k')
    plt.plot([-2, 2], [2, 2], 'k')
    plt.plot([-2, -2], [-2, 2], 'k')
    plt.plot([2, 2], [-2, 2], 'k')
    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim([-2.5, 2.5])
    plt.ylim([-2.5, 2.5])

    plt.show()
