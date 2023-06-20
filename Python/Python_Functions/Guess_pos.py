import numpy as np


def CalcPos(wire_directions, anchor_positions, motor_positions):
    #print('Starting')
    m1 = motor_positions[0, :2]
    m3 = motor_positions[2, :2]
    a1 = anchor_positions[0, :2]
    a3 = anchor_positions[2, :2]
    stepsize = -0.01
    #wire_directions = [[2, -1, 0], [2, 1, 0]]
    w1 = np.multiply(wire_directions[0, :2], stepsize)
    w2 = np.multiply(wire_directions[1, :2], stepsize)
    p1 = a1-m1
    p2 = a3-m3
    #print('p1 equal: '+str(p1))
    while np.all(p1 >= 0):
        #print('p1 equal: '+str(p1))
        #print('p2 equal: '+str(p2))
        #print('dist = '+str(p2[0]-p1[0]))
        if abs(p2[0]-p1[0]) < 0.1:
            COM = p1
            return COM
        elif (p2[0]-p1[0]) < -0.01:
            return 999

        p1 = np.add(p1, w1)
        p2 = np.add(p2, w2)
    return 0


if __name__ == "__main__":
    a1 = [0.0, 3.355, -0.20646]
    a2 = [1.45, 3.355, -0.20646]
    a3 = [2.9, 3.355, -0.20646]
    a4 = [2.9, 0.0, -0.20646]
    a5 = [0.0, 0.0, -0.20646]
    ANCHORS_POS = np.array([a1, a2, a3, a4, a5])  # 5x3 matrix

    m1 = [-0.36785, 0.25047, 0.00996]
    m2 = [0.00065, 0.25047, 0.00996]
    m3 = [0.41995, 0.16827, 0.00996]
    m4 = [0.33960, -0.31953, 0.00996]
    m5 = [-0.45005, -0.23873, 0.00996]
    MOTORS_POS = np.array([m1, m2, m3, m4, m5])  # 5x3 matrix
    fals_motor = np.array([[-1, 1, 0], [0,1,0],[1,1,0],[1,0,0],[0,0,0]])
    wiredata = np.array([[-0.46309844,  0.78822649, -0.40526387], [0.25380909,  0.82838595, -0.49935725]])
    point = CalcPos(wiredata, ANCHORS_POS, MOTORS_POS)
    print(point)
