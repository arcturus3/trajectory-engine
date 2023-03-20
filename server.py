import zmq
import RapidQuadrocopterTrajectories.Python.quadrocoptertrajectory as quadtraj
import generator

def generate(message):
    waypoints = message['waypoints']
    trajectory = generator.generate(
        5,
        waypoints[0]['position'],
        (0, 0, 0),
        (0, 0, 0),
        waypoints[1]['position'],
        (0, 0, 5),
        None
    )
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
        'normal': tuple(trajectory.get_normal_vector(time))
    }

def handle_request(request):
    handlers = {
        'generate_request': generate,
        'query_request': query
    }
    message_type = request['message_type']
    response = handlers[message_type](request)
    return response

trajectories: list[quadtraj.RapidTrajectory] = []

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind('tcp://*:5555')
while True:
    request = socket.recv_json()
    print('received:', request)
    response = handle_request(request)
    socket.send_json(response)
    print('sent:', response)
