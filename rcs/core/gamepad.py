"""A gamepad plugged into the operator PC, for driving.

Same axis handling as the robot's flydigi node (axis numbers differ between
pads, a released trigger rests at -1.0): every axis is sampled while the pad
is untouched, the resting value is subtracted, axes resting near +/-1 are
triggers and never drive, and buttons held at that moment are ignored until
released.

Unlike that node, driving needs a button held down (LB): this pad sits on a
desk, and a knocked stick must not move a robot in another room.

Which axis is which depends on the system, not only on the pad. Linux's
driver gives LX, LY, LT, RX, RY, RT; SDL on Windows (XInput, HIDAPI) gives
LX, LY, RX, RY, LT, RT. Taking axis 3 as "right stick X" everywhere made the
right stick's up/down the turn on Windows, and sideways did nothing: the robot
only went forward and back. So:
  - a pad SDL knows (Xbox, Flydigi, PlayStation, Switch... nearly all) is read
    through SDL's GameController API, which names the axes the same way on
    every system (Controller below);
  - any other pad: the layout is told from where its triggers rest;
  - either stick turns and either drives: left stick alone, or left for speed
    and right for turning, whichever is pushed further.

pygame is only imported when the operator turns the gamepad on.
"""
from __future__ import annotations

import os
import sys
from typing import List, NamedTuple, Optional

# Stick axes (lx, ly, rx, ry) for each layout; triggers come from the calibration.
STANDARD = (0, 1, 2, 3)         # SDL GameController, XInput/HIDAPI on Windows, generic 4-axis pads
LINUX_XPAD = (0, 1, 3, 4)       # Linux xpad / evdev: 2 and 5 are the triggers
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
        self.axis_linear = -1           # the main ones (left stick Y, right stick X)
        self.axis_angular = -1
        self.linear_axes: List[int] = []     # every axis that drives / turns, main one first
        self.angular_axes: List[int] = []
        self.layout = ""
        self._was = {BTN_ESTOP: False, BTN_SPEED: False}

    @property
    def calibrating(self) -> bool:
        return self._ticks < CALIBRATION_TICKS

    def _finish_calibration(self) -> None:
        n = CALIBRATION_TICKS
        self.rest = [s / n for s in self._sums]
        self.triggers = {a for a, r in enumerate(self.rest) if abs(r) > TRIGGER_REST}
        self.masked = {b for b in range(self.pad.get_numbuttons()) if self.pad.get_button(b)}

        self.layout, (lx, ly, rx, ry) = self._layout()

        def usable(axes):
            return [a for a in axes if 0 <= a < len(self.rest) and a not in self.triggers]

        self.linear_axes = usable((ly, ry))
        self.angular_axes = usable((rx, lx))
        self.axis_linear = self.linear_axes[0] if self.linear_axes else -1
        self.axis_angular = self.angular_axes[0] if self.angular_axes else -1

    def _layout(self):
        """(name, (lx, ly, rx, ry)) of this pad."""
        told = getattr(self.pad, "layout", "")
        if told == "standard":
            return "standard", STANDARD
        n, trig = len(self.rest), self.triggers
        if n >= 6 and {2, 5} <= trig:
            return "linux", LINUX_XPAD
        if n >= 6 and {4, 5} <= trig:
            return "standard", STANDARD
        if n == 4:
            return "standard", STANDARD
        # Triggers not found (some rest at 0 until first pressed): the platform's usual order
        return ("linux", LINUX_XPAD) if sys.platform.startswith("linux") and n >= 5 else ("standard", STANDARD)

    def _axis(self, a: int) -> float:
        if a < 0:
            return 0.0
        return _deadzone(max(-1.0, min(1.0, self.pad.get_axis(a) - self.rest[a])))

    def _strongest(self, axes) -> float:
        """The axis pushed furthest of these (0 when none is)."""
        return max((self._axis(a) for a in axes), key=abs, default=0.0)

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
        lin = -self._strongest(self.linear_axes) if deadman else 0.0     # stick up reads negative
        ang = -self._strongest(self.angular_axes) if deadman else 0.0    # stick left reads negative
        return PadInput(lin, ang, deadman, self._pressed(BTN_ESTOP), self._pressed(BTN_SPEED))


class Controller:
    """A pad SDL knows, through its GameController API, read like a joystick
    with a fixed layout: axes LX, LY, RX, RY, LT, RT (-1..1, a released trigger
    -1 as on Linux), buttons A, B, X, Y, LB, RB, Back, Start, Guide, LS, RS."""

    layout = "standard"

    def __init__(self, pg, ctl) -> None:
        self.ctl = ctl
        c = pg
        self._axes = (c.CONTROLLER_AXIS_LEFTX, c.CONTROLLER_AXIS_LEFTY, c.CONTROLLER_AXIS_RIGHTX,
                      c.CONTROLLER_AXIS_RIGHTY, c.CONTROLLER_AXIS_TRIGGERLEFT, c.CONTROLLER_AXIS_TRIGGERRIGHT)
        self._buttons = (c.CONTROLLER_BUTTON_A, c.CONTROLLER_BUTTON_B, c.CONTROLLER_BUTTON_X,
                         c.CONTROLLER_BUTTON_Y, c.CONTROLLER_BUTTON_LEFTSHOULDER,
                         c.CONTROLLER_BUTTON_RIGHTSHOULDER, c.CONTROLLER_BUTTON_BACK,
                         c.CONTROLLER_BUTTON_START, c.CONTROLLER_BUTTON_GUIDE,
                         c.CONTROLLER_BUTTON_LEFTSTICK, c.CONTROLLER_BUTTON_RIGHTSTICK)

    def get_numaxes(self) -> int:
        return len(self._axes)

    def get_axis(self, a: int) -> float:
        v = self.ctl.get_axis(self._axes[a]) / 32767.0
        if a >= 4:                      # triggers: 0..1 from SDL
            v = v * 2.0 - 1.0
        return max(-1.0, min(1.0, v))

    def get_numbuttons(self) -> int:
        return len(self._buttons)

    def get_button(self, b: int) -> bool:
        return bool(self.ctl.get_button(self._buttons[b]))

    def attached(self) -> bool:
        try:
            return bool(self.ctl.attached())
        except Exception:  # noqa: BLE001
            return False

    def quit(self) -> None:
        self.ctl.quit()


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
        ctl = self._controller()
        if ctl is not None:
            self.joy = ctl
            return ctl.ctl.name or "gamepad"
        self.joy = self.pg.joystick.Joystick(0)
        self.joy.init()
        return self.joy.get_name()

    def _controller(self) -> Optional[Controller]:
        """The pad through SDL's GameController API, if SDL knows it."""
        try:
            from pygame._sdl2 import controller
            controller.init()
            if not controller.is_controller(0):
                return None
            return Controller(self.pg, controller.Controller(0))
        except Exception:  # noqa: BLE001 - old pygame, or SDL without the API: plain joystick
            return None

    def pump(self) -> bool:
        """Update the readings; False once the pad is gone."""
        self.pg.event.pump()
        if isinstance(self.joy, Controller):
            return self.joy.attached()
        return self.joy is not None and self.pg.joystick.get_count() > 0

    def close(self) -> None:
        if self.joy is not None:
            self.joy.quit()
            self.joy = None
