import math

from util.orientation import relative_location, Orientation
from util.vec import Vec3
import numpy as np
from RLUtilities.Maneuvers import solve_PWL
from RLUtilities.Mechanics import AerialTurn


class Angles:
    @staticmethod
    def car_location_angle_flattened(car, loc: Vec3):
        car_orientation = Orientation(car.rotation)
        relative_target = relative_location(car.location, car_orientation, loc).flat()
        # car_orientation_flat = car_orientation.forward.flat()
        car_direction = Vec3(10, 0, 0)
        angle_cos = (relative_target.dot(car_direction)) / (car_direction.length() * relative_target.length())
        return math.acos(angle_cos) if relative_target.y < 0 else -math.acos(angle_cos)

    @staticmethod
    def rad_to_deg(angle):
        return (angle / (2 * math.pi)) * 360


class Equations:
    @staticmethod
    def quadratic_formula(a, b, c):
        discriminant = b ** 2 - 4 * a * c
        if discriminant >= 0:
            return (-b - math.sqrt(discriminant)) / 2 * a, (-b + math.sqrt(discriminant)) / 2 * a
        else:
            return None


class Line:
    def __init__(self, start: Vec3, end: Vec3):
        self.start = start
        self.end = end

    def calc_distance(self, loc: Vec3):
        diff = self.end - self.start
        d = diff / diff.length()
        v = loc - self.start
        t = v.dot(d)
        p = self.start + t * d
        return (p - loc).length()


class Plane:
    def __init__(self, normal: Vec3, d, name: str, orientation: str):
        self.normal = normal.normalized()
        self.d = d
        self.name: str = name
        self.type: str = orientation

    def distance_to(self, loc: Vec3):
        return (self.closest_point(loc) - loc).length()

    def closest_point(self, loc: Vec3):
        alpha = (self.d - self.normal.dot(loc)) / (self.normal.dot(self.normal))
        res = loc + alpha * self.normal
        if res.x==res.y==res.z:
            res = Vec3(0.1, 0.1, 0.1)
        return res

    def flatten(self, loc: Vec3):
        res = loc - (loc.dot(self.normal)) * self.normal
        if res.x==res.y==res.z:
            res = Vec3(0.1, 0.1, 0.1)
        return res


# w0: beginning step angular velocity (world coordinates)
# w1: beginning step angular velocity (world coordinates)
# theta: orientation matrix
# dt: time step
def aerial_rpy(w0: np.ndarray, w1: np.ndarray, theta: np.ndarray, dt):
    # car's moment of inertia (spherical symmetry)
    J = 10.5

    # aerial control torque coefficients
    T = np.array([-400.0, -130.0, 95.0])

    # aerial damping torque coefficients
    H = np.array([-50.0, -30.0, -20.0])

    # get angular velocities in local coordinates
    w0_local = w0.dot(theta)
    #w1_local = w1.dot(theta)
    #w0_local = w0
    w1_local = w1

    # PWL equation coefficients
    a = [T[i] * dt / J for i in range(0, 3)]
    b = [-w0_local[i] * H[i] * dt / J for i in range(0, 3)]
    c = [w1_local[i] - (1 + H[i] * dt / J) * w0_local[i] for i in range(0, 3)]

    # RL treats roll damping differently
    b[0] = 0

    return Vec3(
        solve_PWL(a[0], b[0], c[0]),
        solve_PWL(a[1], b[1], c[1]),
        solve_PWL(a[2], b[2], c[2])
    )
