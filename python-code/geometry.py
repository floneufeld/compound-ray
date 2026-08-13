import math

# coordination conversion function
def convert_coor(x,y,z):
    x = float(x)
    y = float(y)
    z = float(z)
    return x, z, -y

# coordinates and yaw for reset to coor0 an look to coor1
def reset(coor0, coor1):

    # 1st xyz
    rx0     = float(coor0[0])
    ry0     = float(coor0[1])
    rz0     = float(coor0[2])

    # 2nd xyz
    rx1     = float(coor1[0])
    ry1     = float(coor1[1])
    rz1     = float(coor1[2])

    # distances to next coordinate
    dx1     = rx1 - rx0
    dy1     = ry1 - ry0
    dz1     = rz1 - rz0

    # yaw facing to the 2nd xyz
    yaw1 = math.atan2(dx1, dz1)

    return rx0, ry0, rz0, yaw1