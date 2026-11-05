"""A gamepad plugged into the operator PC, for driving.

Same axis handling as the robot's flydigi node (axis numbers differ between
pads, a released trigger rests at -1.0): every axis is sampled while the pad
is untouched, the resting value is subtracted, axes resting near +/-1 are
triggers and never drive, and buttons held at that moment are ignored until
released.

Unlike that node, driving needs a button held down (LB): this pad sits on a
desk, and a knocked stick must not move a robot in another room.

pygame is only imported when the operator turns the gamepad on.
"""
from __future__ import annotations

import os
from typing import List, NamedTuple, Optional

LINEAR_AXES = (1, 4)            # left stick Y, then right stick Y
ANGULAR_AXES = (3, 2, 0)        # right stick X, then compact RX, then left stick X
TRIGGER_REST = 0.7
DEADZONE = 0.15
CALIBRATION_TICKS = 20          # read() calls spent learning the resting state

BTN_ESTOP = 1                   # B
BTN_DEADMAN = 4                 # LB, hold to drive
BTN_SPEED = 5                   # RB, next speed step


class PadInput(NamedTuple):
    linear: float               # -1..1, forward positive; 0 unless the dead-man is held
    angular: float              # -1..1, left positive
    deadman: bool
    estop: bool                 # B pressed (only on the press)
    next_speed: bool            # RB pressed (only on the press)


def _deadzone(v: float) -> float:
    if abs(v) < DEADZONE:
        return 0.0
    return (abs(v) - DEADZONE) / (1.0 - DEADZONE) * (1 if v > 0 else -1)


class GamepadReader:
    """Turns raw axis/button reads of a pad into driving input. `pad` needs
    get_numaxes, get_axis, get_numbuttons, get_button."""

    def __init__(self, pad) -> None:
        self.pad = pad
        self._sums = [0.0] * pad.get_numaxes()
        self._ticks = 0
        self.rest: List[float] = []
        self.triggers: set = set()
        self.masked: set = set()
        self.axis_linear = -1
        self.axis_angular = -1
        self._was = {BTN_ESTOP: False, BTN_SPEED: False}

    @property
    def calibrating(self) -> bool:
        return self._ticks < CALIBRATION_TICKS

    def _finish_calibration(self) -> None:
        n = CALIBRATION_TICKS
        self.rest = [s / n for s in self._sums]
        self.triggers = {a for a, r in enumerate(self.rest) if abs(r) > TRIGGER_REST}
        self.masked = {b for b in range(self.pad.get_numbuttons()) if self.pad.get_button(b)}

        def pick(candidates, exclude=-1):
            return next((a for a in candidates if a < len(self.rest) and a not in self.triggers
                         and a != exclude), -1)

        self.axis_linear = pick(LINEAR_AXES)
        self.axis_angular = pick(ANGULAR_AXES, self.axis_linear)

    def _axis(self, a: int) -> float:
        if a < 0:
            return 0.0
        return _deadzone(max(-1.0, min(1.0, self.pad.get_axis(a) - self.rest[a])))

    def _button(self, b: int) -> bool:
        if b >= self.pad.get_numbuttons():
            return False
        on = bool(self.pad.get_button(b))
        if b in self.masked:
            if not on:
                self.masked.discard(b)      # released: trust it from now on
            return False
        return on

    def _pressed(self, b: int) -> bool:
        on = self._button(b)
        edge = on and not self._was[b]
        self._was[b] = on
        return edge

    def read(self) -> PadInput:
        if self.calibrating:
            for a in range(len(self._sums)):
                self._sums[a] += self.pad.get_axis(a)
            self._ticks += 1
            if not self.calibrating:
                self._finish_calibration()
            return PadInput(0.0, 0.0, False, False, False)
        deadman = self._button(BTN_DEADMAN)
        lin = -self._axis(self.axis_linear) if deadman else 0.0     # stick up reads negative
        ang = -self._axis(self.axis_angular) if deadman else 0.0
        return PadInput(lin, ang, deadman, self._pressed(BTN_ESTOP), self._pressed(BTN_SPEED))


class Pygame:
    """The first gamepad found, through pygame/SDL."""

    def __init__(self) -> None:
        # no SDL window, and the pad must keep working while it has no focus
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        self.pg = pygame
        pygame.display.init()
        pygame.joystick.init()
        self.joy = None

    def open(self) -> Optional[str]:
        """Name of the pad, or None if there is none."""
        self.pg.event.pump()
        if self.pg.joystick.get_count() == 0:
            return None
        self.joy = self.pg.joystick.Joystick(0)
        self.joy.init()
        return self.joy.get_name()

    def pump(self) -> bool:
        """Update the readings; False once the pad is gone."""
        self.pg.event.pump()
        return self.joy is not None and self.pg.joystick.get_count() > 0

    def close(self) -> None:
        if self.joy is not None:
            self.joy.quit()
            self.joy = None
