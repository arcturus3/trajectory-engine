import zmq
import numpy as np
from trajectory import Trajectory, Constraint

def generate(message):
    segment_duration = 5
    waypoints = message['waypoints']
    constraints = []
    for i, waypoint in enumerate(waypoints):
        position = np.array(waypoint['position'])
        time = segment_duration * i
        constraint = Constraint(position, time)
        constraints.append(constraint)
    trajectory = Trajectory(constraints)
    trajectory_id = len(trajectories)
    trajectories.append(trajectory)
    return {
        'message_type': 'generate_response',
        'trajectory_id': trajectory_id
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
    response = handlers[message_type](request)
    return response

trajectories: list[Trajectory] = []

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind('tcp://*:5555')
while True:
    request = socket.recv_json()
    print('received:', request)
    response = handle_request(request)
    socket.send_json(response)
    print('sent:', response)
