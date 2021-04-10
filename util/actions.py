###################################
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.action_scheduler import Scheduler
from util.objects import Car, CarDriveTarget, Field
from util.orientation import relative_location, Orientation
from util.vec import Vec3
from util.math import Angles, aerial_rpy
import numpy as np


class Action:
    # abstract class
    # must always return a controller state when terminating
    # if the action terminates, that must mean that the next action is free to begin the next tick. waiting must be
    # handled by the action
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, max_time=15, is_filler=False):
        self.started = False
        self.start_time = 0
        self.elapsed_time = 0
        self.max_time = max_time
        self.is_filler = is_filler
        self.last_controller = None
        self.scheduler = scheduler
        self.completed = False
        self.field = field
        self.car = car
        self.is_aerial_action = False

    def _first_call(self, packet: GameTickPacket):
        self.started = True
        self.start_time = packet.game_info.seconds_elapsed
        if self.is_filler:
            self.max_time = packet.game_info.game_time_remaining
            if packet.game_info.is_unlimited_time:
                self.max_time = 3600

    def _do_always(self, packet: GameTickPacket):
        # start the action
        if not self.started:
            self._first_call(packet)
        # update action runtime
        self.elapsed_time = packet.game_info.seconds_elapsed - self.start_time
        self.update(packet)

    def update(self, packet: GameTickPacket):
        # to define in the action, if any updates need to be made every tick that for some reason
        # should be separated from controller_state
        pass

    def controller_state(self, packet: GameTickPacket):
        # should be called in any Action, filler or not
        self._do_always(packet)
        # must return a valid controller state
        # is responsible for terminating, unless it is a filler action, which may be terminated by get_output
        return SimpleControllerState()


# Action skeleton to copy when creating a new Action. for explanations see class Action above
class ActionSkeleton(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)

    def update(self, packet: GameTickPacket):
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()
        # -------------------------------
        # if exitcondition:
        #     self.scheduler.pop()
        #     if not self.scheduler.is_empty():
        #         return self.scheduler.get_controller_state()
        #     else:
        #         return controller
        # else do logic:
        #   - decide to push some other action
        #       -> return self.scheduler.get_controller_state()
        #   or
        #   - define controller in this action
        # -------------------------------
        return controller


class GroundDriveTowardsFixedAnyState(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, target: CarDriveTarget, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)
        self.is_filler = False

        self.target: CarDriveTarget = target
        # adjust the target to account for our car's offset
        target.adjust_offset(self.car)

    def update(self, packet: GameTickPacket):
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()
        # calculate our angle to the target
        ang_to_target = Angles.rad_to_deg(Angles.car_location_angle_flattened(self.car, self.target.loc))

        distance_to_target = (self.target.loc - self.car.location).length()
        if distance_to_target < 20:  # TODO: adjust exit condition!
            # if we have reached our target, we terminate the action and return the empty base controller
            self.scheduler.pop()
            return controller

        if abs(ang_to_target) >= 0.2:
            # we are not aligned with the target and need to steer towards it
            self.scheduler.push(GroundTurnTowardsFixedAnyState(self.scheduler, self.field, self.car, self.target))
            controller = self.scheduler.get_controller_state(packet)
        else:
            # target is right ahead, no need to steer
            if not self.car.is_super_sonic:
                controller.boost = 1
            controller.throttle = 1

        return controller


class GroundTurnTowardsFixedAnyState(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, target: CarDriveTarget, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)
        self.is_filler = False

        self.target: CarDriveTarget = target

    def update(self, packet: GameTickPacket):
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()

        # calculate angle and distance to target
        ang_to_target = Angles.rad_to_deg(Angles.car_location_angle_flattened(self.car, self.target.loc))
        target_distance = (self.target.loc - self.car.location).length()

        if abs(ang_to_target) < 0.2 or target_distance < 20:
            # we are going towards our target already, no need to steer any more
            self.scheduler.pop()
            controller = self.scheduler.get_controller_state(packet)
        else:
            # we are not aligned with our target and need to steer
            # calculate the optimal direction to turn towards
            turn_direction = -1 if ang_to_target > 0 else 1
            ang_to_target = abs(ang_to_target)

            # in some situations, we might want to turn towards our goal, even if it is a bit of a larger turn
            goal_distance = (self.field.goals[self.car.team].loc - self.car.location).length()
            turn_towards_goal = (goal_distance < 3000 or self.car.location.y < -3500) and goal_distance > 1500
            # TODO: check for enemies and incorporate that in decision making
            if turn_towards_goal and ang_to_target > 130:
                # conditions are met, we decide to turn towards the goal
                goal_loc = self.field.goals[self.car.team].loc
                ang_to_goal = ang_to_target = Angles.rad_to_deg(Angles.car_location_angle_flattened(self.car, self.target.loc))
                turn_direction = -1 if ang_to_goal > 0 else 1

            if ang_to_target > 90:
                # we need to make a sharp turn
                # if self.car.velocity.length() < 350:
                #     # we are going very slowly or even backwards, and the angle to our target is large:
                #     # we decide to execute a half flip towards our target
                #     # TODO: check if we are going backwards
                #     # TODO: half flip backwards
                #     pass
                #
                # else:
                    # let's check if our target is directly behind us
                    target_directly_behind = target_distance < 300 and ang_to_target > 175
                    if target_directly_behind:
                        # the target is directly behind us, it's better to just drive/flip backwards
                        # TODO: flip backwards
                        controller.throttle = -1  # TODO: adjust throttle level
                    else:
                        # the target is not directly behind us and we need to make a sharp turn,
                        # and thus decide to do a handbrake turn
                        controller.throttle = min(1, target_distance/700)  # TODO: adjust throttle level
                        controller.steer = turn_direction
                        controller.handbrake = True
            else:
                # we do not need to turn very strongly
                if ang_to_target > 20:
                    # if our angle to the target is still fairly large but we are already close to the target,
                    # we don't want to go full speed to avoid driving past it
                    controller.throttle = min(1, 0.1 + target_distance/700)  # TODO: adjust throttle level
                else:
                    # nothing to worry about, full throttle
                    controller.throttle = 1
                # we turn towards our target, if the angle is very small we don't steer 100% to avoid overcorrecting
                controller.steer = turn_direction * min(0.1 * ang_to_target, 1)

        return controller


class CarDrive(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, target: CarDriveTarget, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)
        self.is_filler = False

        self.target: CarDriveTarget = target
        # adjust the target to account for our car's offset
        target.adjust_offset(self.car)

    def update(self, packet: GameTickPacket):
        tmp = packet.game_ball.physics.location
        t = Vec3(tmp.x, tmp.y, tmp.z)
        i = 0 if self.car.team == 1 else 1
        delta = (t - self.field.goals[i].loc).normalized() * (self.car.hitbox.w * 0.4 + 93)
        self.target.loc = t + delta
        if self.car.team == 0:
            if self.car.location.y > self.target.loc.y:
                self.target.loc = self.field.goals[self.car.team].loc
        else:
            if self.car.location.y < self.target.loc.y:
                self.target.loc = self.field.goals[self.car.team].loc
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()
        # calculate our angle to the target
        ang_to_target = Angles.rad_to_deg(Angles.car_location_angle_flattened(self.car, self.target.loc))

        distance_to_target = (self.target.loc - self.car.location).length()
        if distance_to_target < 20:  # TODO: adjust exit condition!
            # if we have reached our target, we terminate the action and return the empty base controller
            self.scheduler.pop()
            return controller

        if abs(ang_to_target) >= 0.2:
            # we are not aligned with the target and need to steer towards it
            self.scheduler.push(CarTurn(self.scheduler, self.field, self.car, self.target))
            controller = self.scheduler.get_controller_state(packet)
        else:
            # target is right ahead, no need to steer
            if not self.car.is_super_sonic:
                controller.boost = 1
            controller.throttle = 1

        return controller


class CarTurn(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, target: CarDriveTarget, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)
        self.is_filler = False

        self.target: CarDriveTarget = target

    def update(self, packet: GameTickPacket):
        tmp = packet.game_ball.physics.location
        t = Vec3(tmp.x, tmp.y, tmp.z)
        i = 0 if self.car.team == 1 else 1
        delta = (t - self.field.goals[i].loc).normalized() * (self.car.hitbox.w * 0.4 + 93)
        self.target.loc = t + delta
        if self.car.team == 0:
            if self.car.location.y > self.target.loc.y:
                self.target.loc = self.field.goals[self.car.team].loc
        else:
            if self.car.location.y < self.target.loc.y:
                self.target.loc = self.field.goals[self.car.team].loc
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()

        # calculate angle and distance to target
        ang_to_target = Angles.rad_to_deg(Angles.car_location_angle_flattened(self.car, self.target.loc))
        target_distance = (self.target.loc - self.car.location).length()

        if abs(ang_to_target) < 0.2 or target_distance < 20:
            # we are going towards our target already, no need to steer any more
            self.scheduler.pop()
            controller = self.scheduler.get_controller_state(packet)
        else:
            # we are not aligned with our target and need to steer
            # calculate the optimal direction to turn towards
            turn_direction = -1 if ang_to_target > 0 else 1
            ang_to_target = abs(ang_to_target)

            # in some situations, we might want to turn towards our goal, even if it is a bit of a larger turn
            goal_distance = (self.field.goals[self.car.team].loc - self.car.location).length()
            turn_towards_goal = (goal_distance < 3000 or self.car.location.y < -3500) and goal_distance > 1500
            # TODO: check for enemies and incorporate that in decision making
            if turn_towards_goal and ang_to_target > 130:
                # conditions are met, we decide to turn towards the goal
                goal_loc = self.field.goals[self.car.team].loc
                ang_to_goal = ang_to_target = Angles.rad_to_deg(Angles.car_location_angle_flattened(self.car, self.target.loc))
                turn_direction = -1 if ang_to_goal > 0 else 1

            if ang_to_target > 90:
                # we need to make a sharp turn
                # if self.car.velocity.length() < 350:
                #     # we are going very slowly or even backwards, and the angle to our target is large:
                #     # we decide to execute a half flip towards our target
                #     # TODO: check if we are going backwards
                #     # TODO: half flip backwards
                #     pass
                #
                # else:
                    # let's check if our target is directly behind us
                    target_directly_behind = target_distance < 300 and ang_to_target > 175
                    if target_directly_behind:
                        # the target is directly behind us, it's better to just drive/flip backwards
                        # TODO: flip backwards
                        controller.throttle = -1  # TODO: adjust throttle level
                    else:
                        # the target is not directly behind us and we need to make a sharp turn,
                        # and thus decide to do a handbrake turn
                        controller.throttle = min(1, target_distance/700)  # TODO: adjust throttle level
                        controller.steer = turn_direction
                        controller.handbrake = True
            else:
                # we do not need to turn very strongly
                if ang_to_target > 20:
                    # if our angle to the target is still fairly large but we are already close to the target,
                    # we don't want to go full speed to avoid driving past it
                    controller.throttle = min(1, 0.1 + target_distance/700)  # TODO: adjust throttle level
                else:
                    # nothing to worry about, full throttle
                    controller.throttle = 1
                # we turn towards our target, if the angle is very small we don't steer 100% to avoid overcorrecting
                controller.steer = turn_direction * min(0.1 * ang_to_target, 1)

        return controller


class AerialRecovery(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, target=None, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)
        self.is_aerial_action = True
        self.target: CarDriveTarget = target

    def update(self, packet: GameTickPacket):
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()
        # -------------------------------
        if self.car.has_wheel_contact:
            self.scheduler.pop()
            if not self.scheduler.is_empty():
                return self.scheduler.get_controller_state(packet)
            else:
                return controller
        else:
            wall, time_to_land, arrival_location, arrival_velocity = self.field.get_projected_wall_intersection_car(self.car)
            if self.target is None:
                flat_velocity = wall.flatten(arrival_velocity)
                if flat_velocity.length() == 0:
                    flat_velocity = wall.flatten(Orientation(self.car.rotation).forward)
                self.target = CarDriveTarget(arrival_location + flat_velocity.normalized(), wall)
            # TODO: if given an actual target, calculate the proper vector towards it, flattened correctly on the
            #  wall we'll land on


        # else do logic:
        #   - decide to push some other action
        #       -> return self.scheduler.get_controller_state()
        #   or
        #   - define controller in this action
        # -------------------------------
        return controller


class AerialAlign(Action):
    def __init__(self, scheduler: Scheduler, field: Field, car: Car, target_orientation: Orientation, max_time=15, is_filler=False):
        super().__init__(scheduler, field, car, max_time, is_filler)
        self.target_orientation: Orientation = target_orientation

    def update(self, packet: GameTickPacket):
        pass

    def controller_state(self, packet: GameTickPacket):
        self._do_always(packet)
        controller: SimpleControllerState = SimpleControllerState()
        # -------------------------------
        # calculate exit condition
        if self.car.orientation.get_max_angle_diff(self.target_orientation) < 0.2 and \
                self.car.ang_velocity.length() < 0.02:
            self.scheduler.pop()
            if not self.scheduler.is_empty():
                return self.scheduler.get_controller_state(packet)
            else:
                return controller

        else:
            #rpy: Vec3 = self.car.get_rpy_input_to_target(self.target_orientation)
            rpy: Vec3 = aerial_rpy(self.car.ang_velocity.to_array(),
                                   np.array([0, 0, 5.5]),
                                   self.car.orientation.to_matrix(), 1)
            controller.roll = rpy.x
            controller.pitch = rpy.y
            controller.yaw = rpy.z
        return controller

