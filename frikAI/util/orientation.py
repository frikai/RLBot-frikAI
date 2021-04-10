import math

from util.vec import Vec3
import numpy as np

# This is a helper class for calculating directions relative to your car. You can extend it or delete if you want.
class Orientation:
    """
    This class describes the orientation of an object from the rotation of the object.
    Use this to find the direction of cars: forward, right, up.
    It can also be used to find relative locations.
    """

    def __init__(self, rotation):
        self.yaw = float(rotation.yaw)
        self.roll = float(rotation.roll)
        self.pitch = float(rotation.pitch)

        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)

        self.forward = Vec3(cp * cy, cp * sy, sp)
        self.right = Vec3(cy*sp*sr-cr*sy, sy*sp*sr+cr*cy, -cp*sr)
        self.up = Vec3(-cr*cy*sp-sr*sy, -cr*sy*sp+sr*cy, cp*cr)

    def get_max_angle_diff(self, other):
        af = self.forward.ang_to(other.forward)
        ar = self.right.ang_to(other.right)
        au = self.up.ang_to(other.up)
        return max(af, ar, au)

    def to_matrix(self):
        cr = math.cos(self.roll)
        sr = math.sin(self.roll)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        theta = np.empty((3, 3))

        theta[0][0] = cp * cy
        theta[1][0] = cp * sy
        theta[2][0] = sp

        theta[0][1] = cp * sp * sr - cr * sy
        theta[1][1] = sy * sp * sr + cr * cy
        theta[2][1] = -cp * sr

        theta[0][2] = -cr * cy * sp - sr * sy
        theta[1][2] = -cr * sy * sp + sr * cy
        theta[2][2] = cp * cr

        return theta

    def ang_vel_to(self, other):
        A = self.to_matrix()
        B = other.to_matrix()
        C = B.dot(A.T)
        eig = np.linalg.eig(C)
        axis = None
        for i in range(len(eig[0])):
            if eig[0][i].real == eig[0][i]:
                ev = eig[1]
                axis = Vec3(ev[0][i], ev[1][i], ev[2][i])
        angle = np.arccos((np.trace(C)-1)/2)
        if axis is None:
            axis = self.forward
            angle = 0.001
        return axis.normalized()*angle

    @staticmethod
    def get_rpy(w_start: Vec3, w_end: Vec3):
        T_r = -36.07956616966136
        T_p = -12.14599781908070
        T_y = 8.91962804287785
        D_r = -4.47166302201591
        D_p = -2.798194258050845
        D_y = -1.886491900437232
        t = w_end - w_start
        t_r = t.x
        t_p = t.y
        t_y = t.z
        u_r = (t_r - D_r * w_start.x) / T_r
        u_p = (t_p - D_p * w_start.y)/(T_p + np.sign(t_p - D_p * w_start.y) * w_start.y * D_p)
        u_y = (t_y - D_y * w_start.z)/(T_y - np.sign(t_y - D_y * w_start.z) * w_start.z * D_y)
        return Vec3(u_r, u_p, u_y)


# Sometimes things are easier, when everything is seen from your point of view.
# This function lets you make any location the center of the world.
# For example, set center to your car's location and ori to your car's orientation, then the target will be
# relative to your car!
def relative_location(center: Vec3, ori: Orientation, target: Vec3) -> Vec3:
    """
    Returns target as a relative location from center's point of view, using the given orientation. The components of
    the returned vector describes:

    * x: how far in front
    * y: how far right
    * z: how far above
    """
    x = (target - center).dot(ori.forward)
    y = (target - center).dot(ori.right)
    z = (target - center).dot(ori.up)
    return Vec3(x, y, z)

