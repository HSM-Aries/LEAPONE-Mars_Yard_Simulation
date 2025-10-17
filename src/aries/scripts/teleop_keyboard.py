#!/usr/bin/env python3
"""
Keyboard + WASD teleop for the Aries arm (ROS 2).

Publishes position setpoints (std_msgs/Float64) to the ROS ↔ Gazebo bridge topics:
  /aries/arm_base_joint/cmd_pos
  /aries/arm_shoulder_joint/cmd_pos
  /aries/arm_elbow_joint/cmd_pos
  /aries/arm_gripper_joint/cmd_pos

Also drives the mobile base with geometry_msgs/Twist on:
  /cmd_vel

Default keybindings (tap or hold):
  Arm
    Base        ← 4   6 →   (or ← →)
    Shoulder    ↓ 2   8 ↑   (or ↓ ↑)
    Elbow       ↓ 1   7 ↑
    Gripper     close 3   9 open
  Base (/cmd_vel)
    W/S  forward/back     A/D  rotate left/right     SPACE  stop

Other keys:
  r : reset all joints & velocities to 0
  [ / ] : decrease / increase step sizes (affects joints & speeds)
  h : show help & current values
  q or ESC (real key or glyph ␛) : quit

Safety:
- Joint limits are enforced based on JOINT_LIMITS below.
- Velocity commands are clamped to VEL_LIMITS.
- Use at your own risk on real hardware.

Usage:
  # In a terminal with your ROS 2 workspace sourced and simulation running
  python3 aries_teleop_keyboard.py

Tip: If you want a different namespace or topic names, edit TOPIC_* constants below or use ROS remapping.
"""
from __future__ import annotations

import sys
import termios
import tty
import select
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# ===== Topic names (match your gazebo_bridge.yaml) =====
TOPIC_BASE = "/aries/arm_base_joint/cmd_pos"
TOPIC_SHOULDER = "/aries/arm_shoulder_joint/cmd_pos"
TOPIC_ELBOW = "/aries/arm_elbow_joint/cmd_pos"
TOPIC_GRIPPER = "/aries/arm_gripper_joint/cmd_pos"
TOPIC_CMD_VEL = "/cmd_vel"

# ===== Joint limits pulled from aries/urdf/arm.xacro =====
JOINT_LIMITS = {
    "base": (-3.14, 3.14),
    "shoulder": (-1.918, 1.918),
    "elbow": (-1.748, 1.748),
    "gripper": (-1.942, 1.942),
}

# ===== Velocity limits for /cmd_vel (m/s and rad/s) =====
VEL_LIMITS = {"lin": (-0.7, 0.7), "ang": (-0.5, 0.5)}

@dataclass
class ArmState:
    base: float = 0.0
    shoulder: float = 0.700
    elbow: float = 1.240
    gripper: float = 1.190
    v_lin: float = 0.0   # m/s
    v_ang: float = 0.0   # rad/s

    def clamp(self):
        for j in ("base", "shoulder", "elbow", "gripper"):
            low, high = JOINT_LIMITS[j]
            v = getattr(self, j)
            if v < low:
                setattr(self, j, low)
            elif v > high:
                setattr(self, j, high)
        # clamp velocities
        llo, lhi = VEL_LIMITS["lin"]
        alo, ahi = VEL_LIMITS["ang"]
        self.v_lin = min(max(self.v_lin, llo), lhi)
        self.v_ang = min(max(self.v_ang, alo), ahi)

class ArmDown:
    base: float = 0.0
    shoulder: float = 1.918
    elbow: float = 0.330
    gripper: float = 0.980
    v_lin: float = 0.0   # m/s
    v_ang: float = 0.0   # rad/s

    def clamp(self):
        for j in ("base", "shoulder", "elbow", "gripper"):
            low, high = JOINT_LIMITS[j]
            v = getattr(self, j)
            if v < low:
                setattr(self, j, low)
            elif v > high:
                setattr(self, j, high)
        # clamp velocities
        llo, lhi = VEL_LIMITS["lin"]
        alo, ahi = VEL_LIMITS["ang"]
        self.v_lin = min(max(self.v_lin, llo), lhi)
        self.v_ang = min(max(self.v_ang, alo), ahi)

class ArmStraight:
    base: float = 0.0
    shoulder: float = 0.0
    elbow: float = 0.0
    gripper: float = 0.0
    v_lin: float = 0.0   # m/s
    v_ang: float = 0.0   # rad/s

    def clamp(self):
        for j in ("base", "shoulder", "elbow", "gripper"):
            low, high = JOINT_LIMITS[j]
            v = getattr(self, j)
            if v < low:
                setattr(self, j, low)
            elif v > high:
                setattr(self, j, high)
        # clamp velocities
        llo, lhi = VEL_LIMITS["lin"]
        alo, ahi = VEL_LIMITS["ang"]
        self.v_lin = min(max(self.v_lin, llo), lhi)
        self.v_ang = min(max(self.v_ang, alo), ahi)

class AriesArmTeleop(Node):
    def __init__(self):
        super().__init__("aries_arm_teleop")
        self.pub_base = self.create_publisher(Float64, TOPIC_BASE, 10)
        self.pub_shoulder = self.create_publisher(Float64, TOPIC_SHOULDER, 10)
        self.pub_elbow = self.create_publisher(Float64, TOPIC_ELBOW, 10)
        self.pub_gripper = self.create_publisher(Float64, TOPIC_GRIPPER, 10)
        self.pub_cmdvel = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)

        self.state = ArmState()
        self.step = 0.01       # radians per keypress (arm joints)
        self.gstep = 0.01      # radians per keypress (gripper)
        self.vstep_lin = 0.15  # m/s per keypress (W/S)
        self.vstep_ang = 0.60  # rad/s per keypress (A/D)

        # Timer to continuously republish the current setpoints (20 Hz)
        self.create_timer(1.0 / 20.0, self.publish_state)

        self.print_help()

    def publish_state(self):
        # publish current targets (joints)
        for pub, val in (
            (self.pub_base, self.state.base),
            (self.pub_shoulder, self.state.shoulder),
            (self.pub_elbow, self.state.elbow),
            (self.pub_gripper, self.state.gripper),
        ):
            msg = Float64()
            msg.data = float(val)
            pub.publish(msg)
        # publish robot velocity (cmd_vel)
        tw = Twist()
        tw.linear.x = float(self.state.v_lin)
        tw.angular.z = float(self.state.v_ang)
        self.pub_cmdvel.publish(tw)

    def adjust(self, joint: str, delta: float):
        setattr(self.state, joint, getattr(self.state, joint) + delta)
        self.state.clamp()
        self.status()

    def reset(self):
        self.state = ArmState()
        self.status()

    def down(self):
        self.state = ArmDown()
        self.status()

    def straight(self):
        self.state = ArmStraight()
        self.status()

    def adjust_vel(self, lin: float = 0.0, ang: float = 0.0):
        self.state.v_lin += lin
        self.state.v_ang += ang
        self.state.clamp()
        self.status()

    def stop_vel(self):
        self.state.v_lin = 0.0
        self.state.v_ang = 0.0
        self.status(prefix="STOP ")

    def step_down(self):
        self.step = max(0.005, self.step * 0.5)
        self.gstep = max(0.005, self.gstep * 0.5)
        self.vstep_lin = max(0.01, self.vstep_lin * 0.5)
        self.vstep_ang = max(0.05, self.vstep_ang * 0.5)
        self.status(prefix="Step size ↓ ")

    def step_up(self):
        self.step = min(0.5, self.step * 2.0)
        self.gstep = min(0.5, self.gstep * 2.0)
        self.vstep_lin = min(2.0, self.vstep_lin * 2.0)
        self.vstep_ang = min(3.0, self.vstep_ang * 2.0)
        self.status(prefix="Step size ↑ ")

    def status(self, prefix: str = ""):
        low_high = lambda j: f"[{JOINT_LIMITS[j][0]:.2f},{JOINT_LIMITS[j][1]:.2f}]"
        vlow_high = lambda: (
            f"v=({VEL_LIMITS['lin'][0]:.1f}..{VEL_LIMITS['lin'][1]:.1f})m/s, "
            f"w=({VEL_LIMITS['ang'][0]:.1f}..{VEL_LIMITS['ang'][1]:.1f})rad/s"
        )
        s = (
            f"{prefix}base={self.state.base:.3f}{low_high('base')}  "
            f"shoulder={self.state.shoulder:.3f}{low_high('shoulder')}  "
            f"elbow={self.state.elbow:.3f}{low_high('elbow')}  "
            f"gripper={self.state.gripper:.3f}{low_high('gripper')}  "
            f" | v_lin={self.state.v_lin:.2f} m/s, v_ang={self.state.v_ang:.2f} rad/s ({vlow_high()})  "
            f" | step={self.step:.3f}, gstep={self.gstep:.3f}, vstep=({self.vstep_lin:.2f},{self.vstep_ang:.2f})"
        )
        self.get_logger().info(s)

    def print_help(self):
        banner = r"""
                    Aries Arm Teleop - keybindings
                    Arm joints
                        Base        ← 4   6 →
                        Shoulder    ↓ 2   8 ↑
                        Elbow       ↓ 1   7 ↑
                        Gripper     close 3   9 open

                    Mobile base (/cmd_vel)
                        W/S forward/back   A/D rotate left/right   SPACE stop

                    Other: r reset t downtomap y straight | [ / ] step-/step+ (affects joints & velocities) | h help | q or ESC quit
                    """
        for line in banner.strip("\n").splitlines():
            self.get_logger().info(line)
        self.status()


def get_key(timeout: float = 0.05) -> str | None:
    """Read one key from stdin (non-blocking). Returns None on timeout.
    Normalizes ESC. Supports common arrow-key escape sequences.
    Also accepts the visible glyph U+241B '␛' and treats it as ESC.
    """
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if not dr:
        return None

    ch1 = sys.stdin.read(1)
    # Normalize the visible ESC glyph to real ESC
    if ch1 == "␛":
        ch1 = "\x1b"

    if ch1 != "\x1b":
        return ch1

    # Read a short tail to capture escape sequence variations (CSI, SS3)
    buf = ch1
    for _ in range(6):  # arrow sequences are short; don't block
        dr, _, _ = select.select([sys.stdin], [], [], 0.001)
        if not dr:
            break
        buf += sys.stdin.read(1)

    # Map common arrow variants
    if buf.startswith("\x1b[A") or buf.startswith("\x1bOA"):
        return "ARROW_UP"
    if buf.startswith("\x1b[B") or buf.startswith("\x1bOB"):
        return "ARROW_DOWN"
    if buf.startswith("\x1b[C") or buf.startswith("\x1bOC"):
        return "ARROW_RIGHT"
    if buf.startswith("\x1b[D") or buf.startswith("\x1bOD"):
        return "ARROW_LEFT"

    # Bare ESC or unhandled sequence
    return "\x1b" if buf == "\x1b" else buf


def main():
    rclpy.init()
    node = AriesArmTeleop()

    # Put stdin in raw mode so we can read single characters
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            ch = get_key(timeout=0.05)
            if ch is None:
                continue

            # normalize to lower-case where applicable
            key = ch if ch.startswith("ARROW_") else ch.lower()

            if key in ("\x1b", "␛", "q"):  # ESC (real or glyph) or q
                running = False
                break
            elif key == "h":
                node.print_help()
            elif key == "r":
                node.reset()
            elif key == "t":
                node.down()
            elif key == "y":
                node.straight()
            elif key == "[":
                node.step_down()
            elif key == "]":
                node.step_up()
            # Arm joints
            elif key in ("4", "ARROW_LEFT"):
                node.adjust("base", -node.step)
            elif key in ("6", "ARROW_RIGHT"):
                node.adjust("base", +node.step)
            elif key in ("2", "ARROW_DOWN"):
                node.adjust("shoulder", -node.step)
            elif key in ("8", "ARROW_UP"):
                node.adjust("shoulder", +node.step)
            elif key == "1":
                node.adjust("elbow", -node.step)
            elif key == "7":
                node.adjust("elbow", +node.step)
            elif key == "3":
                node.adjust("gripper", -node.gstep)
            elif key == "9":
                node.adjust("gripper", +node.gstep)
            # Mobile base (/cmd_vel)
            elif key == "w":
                node.adjust_vel(lin=+node.vstep_lin)
            elif key == "s":
                node.adjust_vel(lin=-node.vstep_lin)
            elif key == "a":
                node.adjust_vel(ang=+node.vstep_ang)
            elif key == "d":
                node.adjust_vel(ang=-node.vstep_ang)
            elif key == " ":
                node.stop_vel()
            else:
                # ignore anything else
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        node.get_logger().info("Exiting teleop.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
