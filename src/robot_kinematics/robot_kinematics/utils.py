"""Shared homogeneous transform utilities."""

from numpy import cos, sin, deg2rad, array, concatenate, reshape,zeros

def cosd(degree):
    return cos(deg2rad(degree))

def sind(degree):
    return sin(deg2rad(degree))

def generateH(RotMat, TrasMat):
    RotMat = array(RotMat)
    TrasMat = array(TrasMat)
    if (RotMat.shape != (3,3)):
        raise ValueError("Invalid rotational matrix shape")
    if (TrasMat.shape != (3,)):
        raise ValueError("Invalid traslational matrix shape")
    RotMat = concatenate((RotMat, [[0, 0, 0]]))
    TrasMat = concatenate((TrasMat, [1]))

    return concatenate((RotMat, reshape(TrasMat, (4,1))), axis=1)

def splitH(H):
    return reshape(H[:-1, -1:], (3,)), H[:-1, :-1]

def getTraslation(H, t):
    return (H @ t)[:-1]


def extendTranslation(t):
    t.append(1)
    return t

def IdetityMat(size=3):
    mat = zeros((size,size))

    for i in range(size):
        mat[i][i] = 1

    return mat
