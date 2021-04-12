import time
from random import random

from rlbot.agents.base_agent import BaseAgent
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator, GameInfoState
from rlbot.utils.structures.game_data_struct import Rotator

from util.actions import *
from util.objects import *
from util.orientation import Orientation
from util.vec import Vec3


def rand():
    return random() * np.random.choice([-1, 1])


class FrikAI(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.first_packet = True

    def initialize_agent(self):
        self.first_packet = True
        # Set up information about the boost pads now that the game is active and the info is available

        self.field = Field(self.get_field_info())
        self.scheduler = Scheduler()

        game_info_state = GameInfoState(world_gravity_z=0.0001)
        game_state = GameState(game_info=game_info_state)

        self.set_game_state(game_state)

    def update(self, packet):
        if self.first_packet:
            self.cars = [Car(packet, i) for i in range(packet.num_cars)]
            self.my_car = self.cars[self.index]
            self.ball = Ball(packet.game_ball)
            self.first_packet = False

        for car in self.cars:
            car.update(packet)
        self.ball.update(packet.game_ball)
        self.field.update_boost_pads(packet.game_boosts)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """
        controller: SimpleControllerState = SimpleControllerState()
        self.update(packet)
        self.do_logic(packet)
        if not self.scheduler.is_empty():
            controller = self.scheduler.get_controller_state(packet)
        self.renderer.begin_rendering()
        self.render()
        self.renderer.end_rendering()
        return controller

    def do_logic(self, packet: GameTickPacket):
        if packet.game_info.is_kickoff_pause:  # TODO: and not isinstance(self.scheduler.current_action, Kickoff)
            self.scheduler.clear()
            # self.scheduler.push(Kickoff()) # TODO: create kickoff action :keggw:

        # if self.scheduler.is_empty():
        #     valid = False
        #     x = 0
        #     y = 0
        #     while not valid:
        #         x = randint(-4096, 4096)
        #         y = randint(-5120, 5120)
        #         max_x = 4096 - 1152
        #         max_y = 5120 - 1152
        #         valid = abs(x) < max_x and abs(y) < max_y
        #     tmp_target = Vec3(x, y, 0)
        #     target = CarDriveTarget(tmp_target, self.field.FLOOR[0])
        #     self.scheduler.push(GroundDriveTowardsFixedAnyState(self.scheduler, self.field,
        #                                                         self.my_car, target))

        if self.scheduler.is_empty():
            t_1 = packet.game_ball.physics.location
            target: CarDriveTarget = CarDriveTarget(Vec3(300, 300, 300), self.field.FLOOR[0])
            # print(target.loc)
            # self.scheduler.push(CarDrive(self.scheduler, self.field, self.my_car, target, is_filler=True))
            time.sleep(3)
            ball_state = BallState(physics=Physics(location=Vector3(0, 0, 1000)))
            rr = Vec3(rand(), rand(), rand()) * math.pi
            rav = Vec3(rand(), rand(), rand()).normalized() * rand() * 5.5
            # angular_velocity=Vector3(rav.x, rav.y, rav.z),
            car_state = CarState(physics=Physics(location=Vector3(0, -500, 1000),
                                                 rotation=Rotator(rr.x, rr.y, rr.z)))

            game_state = GameState(ball=ball_state, cars={self.index: car_state})

            self.set_game_state(game_state)
            rotation = Rotator(0, np.pi / 2, 0)
            self.scheduler.push(AerialTest(self.scheduler, self.field, self.my_car, Orientation(rotation)))

    def render(self):
        gray_bg = self.renderer.create_color(230, 125, 125, 125)
        car = self.my_car
        ts = 1
        self.renderer.draw_rect_2d(45, 180, 450 * ts, 200 * ts, True, gray_bg)
        line_nr = 0
        x = 50 + self.team * 1000
        y = 185 + 500 * car.index % 4
        _line_height = 21 * ts
        self.renderer.draw_string_2d(x, y + line_nr * _line_height, ts, ts,
                                     f"{car.name}: {car.score_info.score} points!",
                                     self.renderer.blue() if self.team == 0 else self.renderer.orange())
        line_nr += 1
        if not self.scheduler.is_empty():
            action_stack = self.scheduler.get_stack_string().splitlines()
            for i in range(0, len(action_stack)):
                if i == 0:
                    color = self.renderer.white()
                elif i < len(action_stack) - 1:
                    color = self.renderer.red()
                else:
                    color = self.renderer.green()
                c_action = self.scheduler.current_action()
                suffix = ": " + str(round(c_action.max_time - c_action.elapsed_time, 2)) if i == len(
                    action_stack) - 1 else ""
                self.renderer.draw_string_2d(x, y + line_nr * _line_height, ts, ts, action_stack[i] + suffix, color)
                line_nr += 1
            current: AerialTest = self.scheduler.action_stack[0]
            c_loc = self.my_car.location
            self.renderer.draw_line_3d(c_loc, c_loc + current.target_orientation.forward * 200, self.renderer.white())
            self.renderer.draw_line_3d(c_loc, c_loc + current.target_orientation.right * 200, self.renderer.white())
            self.renderer.draw_line_3d(c_loc, c_loc + current.target_orientation.up * 200, self.renderer.white())
            # tmp = self.scheduler.current_action().target.loc
            # self.renderer.draw_line_3d(tmp, tmp + Vec3(0, 0, 300), self.renderer.white())
            # self.renderer.draw_rect_3d(tmp + Vec3(0, 0, 300), 30, 30, True, self.renderer.pink(), centered=True)
            # distance = round((tmp - self.my_car.location).length(), 2)
            # self.renderer.draw_string_2d(x, y + line_nr * _line_height, ts, ts,
            #                              str(distance), self.renderer.green() if distance < 5 else self.renderer.red())
            # line_nr += 1

    # this is old code that was part of PythonExampleBot. we can do better :yay: (I hope)
    # def begin_front_flip(self, packet):
    #     # Send some quickchat just for fun
    #     self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)
    #
    #     # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
    #     # logic during that time because we are setting the active_sequence.
    #     self.active_sequence = Sequence([
    #         ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
    #         ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
    #         ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
    #         ControlStep(duration=0.8, controls=SimpleControllerState()),
    #     ])
    #
    #     # Return the controls associated with the beginning of the sequence so we can start right away.
    #     return self.active_sequence.tick(packet)
    #
    #
    #
    #     # Keep our boost pad info updated with which pads are currently active
    #     self.boost_pad_tracker.update_boost_status(packet)
    #
    #     # This is good to keep at the beginning of get_output. It will allow you to continue
    #     # any sequences that you may have started during a previous call to get_output.
    #     if self.active_sequence is not None and not self.active_sequence.done:
    #         controls = self.active_sequence.tick(packet)
    #         if controls is not None:
    #             return controls
    #
    #     # Gather some information about our car and the ball
    #     my_car = packet.game_cars[self.index]
    #     car_location = Vec3(my_car.physics.location)
    #     car_velocity = Vec3(my_car.physics.velocity)
    #     ball_location = Vec3(packet.game_ball.physics.location)
    #
    #     # By default we will chase the ball, but target_location can be changed later
    #     target_location = ball_location
    #
    #     if car_location.dist(ball_location) > 1500:
    #         # We're far away from the ball, let's try to lead it a little bit
    #         ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
    #         ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + 2)
    #
    #         # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
    #         # replays, so check it to avoid errors.
    #         if ball_in_future is not None:
    #             target_location = Vec3(ball_in_future.physics.location)
    #             self.renderer.draw_line_3d(ball_location, target_location, self.renderer.cyan())
    #
    #     # Draw some things to help understand what the bot is thinking
    #     self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
    #     self.renderer.draw_string_3d(car_location, 1, 1, f'Speed: {car_velocity.length():.1f}', self.renderer.white())
    #     self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)
    #
    #
    #     if 750 < car_velocity.length() < 800:
    #         # We'll do a front flip if the car is moving at a certain speed.
    #         return self.begin_front_flip(packet)
    #
    #     controls = SimpleControllerState()
    #     controls.steer = steer_toward_target(my_car, target_location)
    #     controls.throttle = 1.0
    #     # You can set more controls if you want, like controls.boost.
    #     self.renderer.end_rendering()
    #     return controls
