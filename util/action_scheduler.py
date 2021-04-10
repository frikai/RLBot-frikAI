from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


class Scheduler:
    def __init__(self):
        self.action_stack = []

    def push(self, action):
        self.action_stack.append(action)
        action.scheduler = self

    def pop(self):
        if not self.is_empty():
            self.action_stack.pop()

    def is_empty(self):
        return len(self.action_stack) == 0

    def clear(self):
        self.action_stack = []

    def size(self):
        return len(self.action_stack)

    def current_action(self):
        if not self.is_empty():
            return self.action_stack[-1]

    def get_controller_state(self, packet: GameTickPacket):
        controller: SimpleControllerState = self.action_stack[-1].controller_state(packet)
        if not self.is_empty():
            current_action = self.action_stack[-1]
            if current_action.completed or (current_action.elapsed_time >= current_action.max_time
                                            and not current_action.is_filler):
                self.pop()
        return controller

    def get_stack_string(self):
        out = "ActionStack:"
        if not self.is_empty():
            if self.size() > 4:
                out += "\n..."
            lines = min(len(self.action_stack) + 1, 5)
            for i in reversed(range(1, lines)):
                out += "\n" + "  "*(lines-i) + "=>" + self.action_stack[-i].__class__.__name__
        else:
            out += " empty"

        return out
