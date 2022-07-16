import math

class OrientationEuler:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        
    def __init__(self, angleX, angleY, angleZ):
        self.x = angleX
        self.y = angleY
        self.z = angleZ

    def getAngleX():
        return self.x

    def getAngleY():
        return self.y

    def getAngleZ():
        return self.z

    def toString(self):
        return "(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) +")"

def QuaternionToEuler(quaternion):
    roll_x = math.atan2(2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y))
    
    val = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    val = 1.0 if val > 1.0 else val
    val = -1.0 if val < -1.0 else val
    pitch_y = math.asin(val)

    yaw_z = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x *quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    return OrientationEuler(roll_x, pitch_y, yaw_z)

def RoundTo(number, round_to):
    base = (int(number/round_to)) * round_to
    if number - base >= float(round_to / 2.0):
        return base + round_to
    else:
        return base