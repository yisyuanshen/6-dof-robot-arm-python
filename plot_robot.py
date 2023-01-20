import matplotlib.pyplot as plt
import numpy as np
import cv2

def plot(pos):
    fig = plt.figure('Robot Confirm')
    ax = fig.add_subplot(projection='3d')
    
    pos = np.array(pos)
    x = pos[:, 0]
    y = pos[:, 1]
    z = pos[:, 2]

    ax.plot3D(x, y, z, color = 'gray', markerfacecolor = 'black', markeredgecolor = 'black',\
                marker = 'p', linewidth=2, markersize=5)

    # Make legend, set axes limits and labels

    ax.set_xlim(-600, 600)
    ax.set_ylim(-600, 600)
    ax.set_zlim(0, 1200)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # ax.view_init(elev=0, azim=0, roll=0)
    # plt.show()

    fig.canvas.draw()
    img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

    return img