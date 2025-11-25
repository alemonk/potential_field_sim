# controller/robot.py
import math
from controller.geometry import quantize, wrapToPi
from controller.potential_field import computePotentialFieldAvoidance

class DifferentialDriveRobot:
    def __init__(self, x, y, yaw, wheel_base):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.wheel_base = float(wheel_base)
        self.vL = 0.0
        self.vR = 0.0
        self.abs_speed = 0.0
        self.stop_simulation = False

    def step(self, target, obstacles, dt):
        outL, outR, stop_simulation = computePotentialFieldAvoidance(
            self.x, self.y, self.yaw,
            target[0], target[1],
            obstacles,
            self.vL, self.vR,
            self.wheel_base
        )
        self.vL = quantize(outL, 0.01)
        self.vR = quantize(outR, 0.01)
        self.abs_speed = (outL + outR) / 2

        v = 0.5 * (self.vL + self.vR)
        omega = (self.vR - self.vL) / self.wheel_base

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = wrapToPi(self.yaw + omega * dt)

        self.stop_simulation = stop_simulation

        return (self.x, self.y, self.yaw, self.vL, self.vR, self.abs_speed, self.stop_simulation)
