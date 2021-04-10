from util.math import Plane, Equations
from util.orientation import Orientation
from util.vec import *
from rlbot.utils.structures.game_data_struct import GameTickPacket, BoxShape, FieldInfoPacket, BoostPadState
from rlbot.utils.structures.game_data_struct import BoostPad as _BoostPad


class Cnst:
    car_mass = 180

    gravity = 650


class Car:
    def __init__(self, packet: GameTickPacket, index: int):
        self.index = index
        this_car = packet.game_cars[self.index]
        self.jump_time = -1
        self.is_bot = this_car.is_bot
        self.name = this_car.name
        self.team = this_car.team
        self.team_string = "blue" if self.team == 0 else "red"
        self.hitbox: RectHitbox = RectHitbox(this_car.hitbox, offset=Vec3(this_car.hitbox_offset))
        self.score_info = this_car.score_info  # score, goals, own_goals, assists, saves, shots, demolitions
        self.double_jumped = this_car.double_jumped
        self.orientation = Orientation(this_car.physics.rotation)

        self.update(packet)

    def update(self, packet: GameTickPacket.game_cars):
        this_car = packet.game_cars[self.index]
        self.location = Vec3(this_car.physics.location)
        self.rotation = this_car.physics.rotation
        self.velocity = Vec3(this_car.physics.velocity)
        self.ang_velocity = Vec3(this_car.physics.angular_velocity)
        self.is_demolished = this_car.is_demolished
        self.has_wheel_contact = this_car.has_wheel_contact
        self.is_super_sonic = this_car.is_super_sonic
        self.jumped = this_car.jumped
        self.double_jumped = this_car.double_jumped or self.double_jumped
        self.boost = this_car.boost
        self.orientation = Orientation(this_car.physics.rotation)
        if self.jumped and self.jump_time == -1:
            self.jump_time = packet.game_info.seconds_elapsed
        if self.jump_time - packet.game_info.seconds_elapsed >= 1.5:
            self.double_jumped = True
            if not self.jumped:
                self.double_jumped = False
                self.jump_time = -1

    def get_rpy_input_to_target(self, target_orientation: Orientation):
        co = self.orientation
        to = target_orientation
        w0 = self.ang_velocity
        w1 = co.rotation_diff_to(to)
        rpy = Orientation.get_rpy(w0, w1)
        angle = rpy.length()
        rpy = rpy.normalized()
        max_scalable = min(w1.length()/5.5, 1 / max(abs(rpy.x), abs(rpy.y), abs(rpy.z)))
        return rpy * min(angle/np.pi, max_scalable)


class Field:
    def __init__(self, field_info: FieldInfoPacket):
        # currently only supports 2 fixed goals and goal locations (hard coded positions for standard maps)
        # TODO: parameterize goal creation based on FieldInfoPacket
        self.NUM_BOOSTS = field_info.num_boosts
        self.goals = [Goal(0), Goal(1)]
        self.boost_pads = [BoostPad(field_info.boost_pads[i]) for i in range(self.NUM_BOOSTS)]
        self.FIELD_VALS = {
            "floor": 0,
            "center": Vec3(0, 0, 0),
            "field_max_x": 4096,
            "side_wall_length": 7936,
            "field_max_y": 5120,
            "back_wall_length": 5888,
            "field_max_z": 2044,
            "goal_height": 642.775,
            "goal_center_to_post": 892.755,
            "goal_depth": 880,
            "corner_wall_length": 1629.174,
            "corner_missing_segment": 1152,  # the length of side/back wall that's missing
            "corner_intersect_x": 8064  # 45° angle intersection point of two corner wall planes
        }
        self.FLOOR = [Plane(Vec3(0, 0, 1), 0, "floor", "horizontal")]
        # +x before -x
        self.SIDE_WALLS = [
            Plane(Vec3(1, 0, 0), 4096, "+x_wall", "vertical"),
            Plane(Vec3(-1, 0, 0), 4096, "-x_wall", "vertical")
        ]
        # -y before +y (blue before red)
        self.BACK_WALLS = [
            Plane(Vec3(0, -1, 0), 5120, "-y_wall", "vertical"),
            Plane(Vec3(0, 1, 0), 5120, "+y_wall", "vertical")
        ]
        # blue +x, blue -x, orange +x, orange -x
        self.CORNER_WALLS = [
            Plane(Vec3(1, -1, 0), 8064, "+x_-y_corner", "vertical"),
            Plane(Vec3(-1, -1, 0), 8064, "-x_-y_corner", "vertical"),
            Plane(Vec3(1, 1, 0), 8064, "+x_+y_corner", "vertical"),
            Plane(Vec3(-1, 1, 0), 8064, "-x_+y_corner", "vertical")
        ]
        self.CEILING = [Plane(Vec3(0, 0, 1), 2044, "ceiling", "horizontal")]
        tmp = [
            self.FLOOR,
            self.SIDE_WALLS,
            self.BACK_WALLS,
            self.CORNER_WALLS,
            self.CEILING
        ]
        self.ALL_WALLS = [plane for sublist in tmp for plane in sublist]

    def update_boost_pads(self, game_boosts: GameTickPacket.game_boosts):
        for i in range(self.NUM_BOOSTS):
            self.boost_pads[i].update(game_boosts[i])

    def get_closest_wall_car_target(self, loc: Vec3):
        closest_point = None
        min_distance = 50000
        plane = None
        for wall in self.ALL_WALLS[:-1]:
            cp = wall.closest_point(loc)
            md = (cp - loc).length()
            if md < min_distance:
                closest_point = cp
                min_distance = md
                plane = wall
        return CarDriveTarget(closest_point, plane)

    def get_projected_wall_intersection_car(self, car: Car):
        v_0 = car.velocity
        a_h = Vec3(0, 0, -Cnst.gravity) * 0.5
        wall = None
        min_time = 100
        arrival_location = None
        arrival_velocity = None
        for plane in self.ALL_WALLS:
            s_0 = car.location - plane.normal * car.hitbox.offset
            a = plane.normal.dot(a_h)
            b = plane.normal.dot(v_0)
            c = plane.normal.dot(s_0) - plane.d
            x_0_1 = Equations.quadratic_formula(a, b, c)
            if x_0_1 is not None:
                m = x_0_1[0] if x_0_1[0] >= 0 else x_0_1[1]
                if m < min_time:
                    min_time = m
                    wall = plane
        if wall is not None:
            s_0 = car.location - wall.normal * car.hitbox.offset
            arrival_location = s_0 + v_0 * min_time + a_h * min_time**2
            arrival_velocity = v_0 + min_time * 2 * a_h
        return wall, min_time, arrival_location, arrival_velocity


class BoostPad:
    def __init__(self, boost_pad: _BoostPad):
        self.large = boost_pad.is_full_boost
        self.loc = boost_pad.location
        self.is_active = True
        self.timer = 0.0

    def update(self, state: BoostPadState):
        self.is_active = state.is_active
        self.timer = state.timer


class Hitbox:
    def __init__(self, offset: Vec3):
        self.offset = offset


class RectHitbox(Hitbox):
    def __init__(self, size: BoxShape, offset: Vec3 = Vec3()):
        super().__init__(offset)
        self.l = size.length
        self.w = size.width
        self.h = size.height


class Ball:
    def __init__(self, game_ball: GameTickPacket.game_ball):
        self.location = None
        self.rotation = None
        self.velocity = None
        self.angular_velocity = None
        self.latest_touch = None
        self.collision_shape = None
        self.update(game_ball)

    def update(self, game_ball: GameTickPacket.game_ball):
        self.location = Vec3(game_ball.physics.location)
        self.rotation = game_ball.physics.rotation
        self.velocity = game_ball.physics.velocity
        self.angular_velocity = game_ball.physics.angular_velocity
        self.latest_touch = game_ball.latest_touch
        self.collision_shape = game_ball.collision_shape


class Target:
    def __init__(self, loc: Vec3):
        self.loc = loc


class CarDriveTarget(Target):
    def __init__(self, loc: Vec3, plane: Plane):
        super().__init__(loc)
        self.plane: Plane = plane

    def adjust_offset(self, car: Car):
        self.loc += self.plane.normal * car.hitbox.offset.z


class BallTarget(Target):
    def __init__(self, loc: Vec3):
        super().__init__(loc)


class Goal(BallTarget):
    def __init__(self, team: int):
        # center of the goal line plane
        super().__init__(Vec3(0, 5120 if team == 1 else -5120, 642.775 / 2))

        self.team = team
        self.half_width = 893
        self.half_height = 321.3875
        self.width = 1786
        self.height = 642.775
        self.depth = 880
        self.post1 = Vec3(self.loc.x - self.half_width,
                          self.loc.y,
                          0)
        self.post2 = Vec3(self.loc.x + self.half_width,
                          self.loc.y,
                          0)

    def get_far_post(self, loc: Vec3):
        return self.post1 if loc.dist(self.post1) > loc.dist(self.post2) else self.post2

    def get_near_post(self, loc: Vec3):
        return self.post1 if loc.dist(self.post1) < loc.dist(self.post2) else self.post2
