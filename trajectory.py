import numpy as np
from RapidQuadrocopterTrajectories.Python.quadrocoptertrajectory import RapidTrajectory as TrajectorySegment

class Constraint:
    def __init__(self, position: np.ndarray, time: float):
        self.position = position
        self.time = time

class Trajectory:
    def __init__(self, constraints: list[Constraint]):
        self.constraints = constraints
        self.f_min = 0
        self.f_max = 25
        self.w_max = 20
        self.gravity = np.array([0, 0, -9.81])
        self.ground_point = np.zeros(3)
        self.ground_normal = np.array([0, 0, 1])
        self.min_time_interval = 0.01
        self.generate()

    def generate(self):
        self.segments: list[TrajectorySegment] = []
        p0 = self.constraints[0].position
        v0 = np.zeros(3)
        a0 = np.zeros(3)
        for i in range(1, len(self.constraints)):
            p1 = self.constraints[i].position
            v1 = np.array([None, None, None])
            a1 = np.array([None, None, None])
            duration = self.constraints[i].time - self.constraints[i - 1].time
            segment = self.generate_segment(p0, v0, a0, p1, v1, a1, duration)
            self.segments.append(segment)
            p0 = segment.get_position(duration)
            v0 = segment.get_velocity(duration)
            a0 = segment.get_acceleration(duration)

    def generate_segment(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        a0: np.ndarray,
        p1: np.ndarray,
        v1: np.ndarray,
        a1: np.ndarray,
        duration: float
    ) -> TrajectorySegment:
        segment = TrajectorySegment(p0, v0, a0, self.gravity)
        segment.set_goal_position(p1)
        segment.set_goal_velocity(v1)
        segment.set_goal_acceleration(a1)
        segment.generate(duration)
        return segment

    def get_segment_time(self, time: float):
        i = len(self.constraints) - 1 # initial segment constraint index
        while time < self.constraints[i].time:
            i -= 1
        if i == len(self.constraints) - 1:
            i -= 1
        segment = self.segments[i]
        segment_time = time - self.constraints[i].time
        return (segment, segment_time)

    def get_position(self, time: float):
        segment, segment_time = self.get_segment_time(time)
        return segment.get_position(segment_time)

    def get_normal(self, time: float):
        segment, segment_time = self.get_segment_time(time)
        return segment.get_normal_vector(segment_time)
