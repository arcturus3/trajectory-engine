import RapidQuadrocopterTrajectories.Python.quadrocoptertrajectory as quadtraj

def generate(
    duration,
    pos_initial,
    vel_initial,
    acc_initial,
    pos_final=None,
    vel_final=None,
    acc_final=None,
):
    gravity = (0, 0, -9.81)
    f_min = 0
    f_max = 25
    w_max = 20
    min_time_interval = 0.01
    ground_point = (0, 0, 0)
    ground_normal = (0, 0, 1)

    traj = quadtraj.RapidTrajectory(pos_initial, vel_initial, acc_initial, gravity)
    if pos_final is not None:
        traj.set_goal_position(pos_final)
    if vel_final is not None:
        traj.set_goal_velocity(vel_final)
    if acc_final is not None:
        traj.set_goal_acceleration(acc_final)

    traj.generate(duration)

    feasible_inputs = traj.check_input_feasibility(f_min, f_max, w_max, min_time_interval)
    feasible_position = traj.check_position_feasibility(ground_point, ground_normal)
    return traj
