import numpy as np
from scipy.spatial.transform import Rotation
from RapidQuadrocopterTrajectories.Python.quadrocoptertrajectory import RapidTrajectory as TrajectorySegment, InputFeasibilityResult

# minimize total acceleration
# define constraint durations instead of times
# optimize over durations instead of fixing user durations
# interpolate between yaws at current final and next final constraints

class Constraint:
    def __init__(self, position: np.ndarray, rotation: np.ndarray | None, time: float):
        self.position = position
        self.rotation = rotation
        self.time = time

class Trajectory:
    def __init__(self, constraints: list[Constraint]):
        self.constraints = constraints
        self.f_min = 0
        self.f_max = 35
        self.w_max = 20
        self.gravity = np.array([0, 0, -9.81])
        self.ground_point = np.zeros(3)
        self.ground_normal = np.array([0, 0, 1])
        self.min_time_interval = 0.01
        self.segments: list[TrajectorySegment] = []

    # generate segments with unconstrained final states
    def generate_naive(self) -> bool:
        self.segments = []
        p0 = self.constraints[0].position
        v0 = np.zeros(3)
        a0 = np.zeros(3)
        for i in range(1, len(self.constraints)):
            p1 = self.constraints[i].position
            v1 = np.full(3, None)
            a1 = np.full(3, None)
            duration = self.constraints[i].time - self.constraints[i - 1].time
            segment, feasible = self.generate_segment(p0, v0, a0, p1, v1, a1, duration)
            if not feasible:
                self.segments = []
                return False
            self.segments.append(segment)
            p0 = segment.get_position(duration)
            v0 = segment.get_velocity(duration)
            a0 = segment.get_acceleration(duration)
        return True

    # generate segments with search over final states to minimize cost of current and next segment
    def generate_greedy(self) -> bool:
        self.segments = []
        p0 = self.constraints[0].position
        v0 = np.zeros(3)
        a0 = np.zeros(3)
        for i in range(1, len(self.constraints)):
            p1 = self.constraints[i].position
            a1 = np.full(3, None)
            duration = self.constraints[i].time - self.constraints[i - 1].time
            if i < len(self.constraints) - 1:
                p1_ind = self.constraints[i + 1].position
                v1_ind = np.full(3, None)
                a1_ind = np.full(3, None)
                best_segment = None
                best_cost = float('inf')
                for direction in self.get_directions(p0, p1, p1_ind):
                    for speed in self.get_speeds():
                        v1 = speed * direction
                        segment, segment_feasible = self.generate_segment(p0, v0, a0, p1, v1, a1, duration)
                        p0_ind = segment.get_position(duration)
                        v0_ind = segment.get_velocity(duration)
                        a0_ind = segment.get_acceleration(duration)
                        induced_segment, induced_segment_feasible = self.generate_segment(p0_ind, v0_ind, a0_ind, p1_ind, v1_ind, a1_ind, duration)
                        if segment_feasible and induced_segment_feasible:
                            cost = segment.get_cost() + induced_segment.get_cost()
                            if cost < best_cost:
                                best_segment = segment
                                best_cost = cost
                segment = best_segment
            else:
                v1 = np.zeros(3)
                segment, segment_feasible = self.generate_segment(p0, v0, a0, p1, v1, a1, duration)
                if not segment_feasible:
                    segment = None
            if segment is None:
                self.segments = []
                return False
            self.segments.append(segment)
            p0 = segment.get_position(duration)
            v0 = segment.get_velocity(duration)
            a0 = segment.get_acceleration(duration)
        return True

    # rotation = normal
    # rotation = thrust / ||thrust||
    # rotation = undefined if thrust = 0
    # acceleration = thrust + gravity
    # acceleration = ||thrust|| * rotation + gravity

    # convert euler angles to normal vector
    # beware of coordinate system change
    def generate_greedy_with_rotation(self) -> bool:
        self.segments = []
        p0 = self.constraints[0].position
        v0 = np.zeros(3)
        a0 = np.zeros(3) # either enforce initial and final accelerations to be 0 or allow rotation to take effect with minimum thrust here
        for i in range(1, len(self.constraints)):
            p1 = self.constraints[i].position
            a1 = np.full(3, None)
            duration = self.constraints[i].time - self.constraints[i - 1].time
            if i < len(self.constraints) - 1:
                p1_ind = self.constraints[i + 1].position
                v1_ind = np.full(3, None)
                a1_ind = np.full(3, None)
                best_segment = None
                best_cost = float('inf')
                for thrust in self.get_thrusts():
                    if self.constraints[i].rotation is not None:
                        rotation = Rotation.from_matrix(self.constraints[i].rotation)
                        normal = rotation.apply(np.array([0, 0, 1]))
                        a1 = thrust * normal + self.gravity
                    for direction in self.get_directions(p0, p1, p1_ind):
                        for speed in self.get_speeds():
                            v1 = speed * direction
                            segment, segment_feasible = self.generate_segment(p0, v0, a0, p1, v1, a1, duration)
                            p0_ind = segment.get_position(duration)
                            v0_ind = segment.get_velocity(duration)
                            a0_ind = segment.get_acceleration(duration)
                            duration_ind = self.constraints[i + 1].time - self.constraints[i].time
                            induced_segment, induced_segment_feasible = self.generate_segment(p0_ind, v0_ind, a0_ind, p1_ind, v1_ind, a1_ind, duration_ind)
                            if segment_feasible and induced_segment_feasible:
                                cost = segment.get_cost() + induced_segment.get_cost()
                                if cost < best_cost:
                                    best_segment = segment
                                    best_cost = cost
                segment = best_segment
            else:
                v1 = np.zeros(3)
                segment, segment_feasible = self.generate_segment(p0, v0, a0, p1, v1, a1, duration)
                if not segment_feasible:
                    segment = None
            if segment is None:
                self.segments = []
                return False
            self.segments.append(segment)
            p0 = segment.get_position(duration)
            v0 = segment.get_velocity(duration)
            a0 = segment.get_acceleration(duration)
        return True

    def generate_segment(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        a0: np.ndarray,
        p1: np.ndarray,
        v1: np.ndarray,
        a1: np.ndarray,
        duration: float
    ) -> tuple[TrajectorySegment, bool]:
        segment = TrajectorySegment(p0, v0, a0, self.gravity)
        segment.set_goal_position(p1)
        segment.set_goal_velocity(v1)
        segment.set_goal_acceleration(a1)
        segment.generate(duration)
        feasibility_result = segment.check_input_feasibility(self.f_min, self.f_max, self.w_max, self.min_time_interval)
        feasible = feasibility_result == InputFeasibilityResult.Feasible
        return (segment, feasible)

    def normalize(self, v: np.ndarray):
        return v / np.linalg.norm(v)

    def get_thrusts(self):
        # return max of this and some small epsilon to ensure that quad is thrusting some amount
        return np.linspace(self.f_min, self.f_max, 30)

    # direction set to optimize over
    def get_directions(self, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray):
        a = self.normalize(p1 - p0)
        b = self.normalize(p2 - p1)
        axis = self.normalize(np.cross(a, b))
        angles = np.linspace(0, 2 * np.pi, 25, endpoint=False)
        directions = []
        for angle in angles:
            rotation = Rotation.from_rotvec(angle * axis)
            directions.append(rotation.apply(a))
        return np.array(directions)

    # speed set to optimize over
    def get_speeds(self):
        return np.linspace(0, 25, 50)

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
