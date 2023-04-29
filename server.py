import zmq
import numpy as np
import traceback
from trajectory import Trajectory, Constraint

def generate(message):
    waypoints = message['waypoints']
    constraints = []
    for waypoint in waypoints:
        position = np.array(waypoint['position'])
        constrain_rotation = waypoint['constrain_rotation']
        rotation = np.array(waypoint['rotation']) if constrain_rotation else None
        time = waypoint['time']
        constraint = Constraint(position, rotation, time)
        constraints.append(constraint)
    trajectory = Trajectory(constraints)
    feasible = trajectory.generate_naive()
    # feasible = trajectory.generate_greedy_with_rotation()
    trajectory_id = len(trajectories)
    trajectories.append(trajectory)
    return {
        'message_type': 'generate_response',
        'trajectory_id': trajectory_id,
        'feasible': feasible
    }

def query(message):
    trajectory_id = message['trajectory_id']
    time = message['time']
    trajectory = trajectories[trajectory_id]
    return {
        'message_type': 'query_response',
        # reparameterize by arc length instead?
        'position': tuple(trajectory.get_position(time)),
        'normal': tuple(trajectory.get_normal(time))
    }

def handle_request(request):
    handlers = {
        'generate_request': generate,
        'query_request': query
    }
    message_type = request['message_type']
    try:
        response = handlers[message_type](request)
    except Exception:
        print(traceback.format_exc())
        response = {
            'error': 'responding to prevent blocking'
        }
    return response

trajectories: list[Trajectory] = []
debug = False

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind('tcp://*:5555')
while True:
    request = socket.recv_json()
    if debug:
        print('received:', request)
    response = handle_request(request)
    socket.send_json(response)
    if debug:
        print('sent:', response)
