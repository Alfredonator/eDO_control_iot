from enum import Enum


class Button(Enum):
    NONE = 0
    UP = 1
    DOWN = 2
    RIGHT = 3
    LEFT = 4
    PLUS = 5
    MINUS = 6
    ENTER = 7
    ESC = 8


class StateWrapper:
    def __init__(self, state):
        self.state = state
