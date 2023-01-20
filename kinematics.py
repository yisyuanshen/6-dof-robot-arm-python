import numpy as np

def forward_T(a, alpha, theta, d):
    alpha = alpha/180*np.pi
    theta = theta/180*np.pi

    T = [[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
         [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
         [0,              np.sin(alpha),                np.cos(alpha),               d              ],
         [0,              0,                            0,                           1              ]]

    return np.matrix(T)


def inverse_R(rx, ry, rz):

    R_z = np.matrix([[np.cos(rz), -np.sin(rz), 0],[np.sin(rz), np.cos(rz), 0],[0, 0, 1]])
    R_y = np.matrix([[np.cos(ry), 0, np.sin(ry)],[0, 1, 0],[-np.sin(ry), 0, np.cos(ry)]])
    R_x = np.matrix([[1, 0, 0],[0, np.cos(rx), -np.sin(rx)],[0, np.sin(rx), np.cos(rx)]])

    R = R_z*R_y*R_x

    return np.matrix(R)


