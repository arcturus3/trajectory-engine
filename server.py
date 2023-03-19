import zmq
import RapidQuadrocopterTrajectories.Python.quadrocoptertrajectory as quadtraj
import generator

def generate(args):
    waypoints = args['waypoints']
    trajectory = generator.generate(
        5,
        waypoints[0]['position'],
        (0, 0, 0),
        (0, 0, 0),
        waypoints[1]['position'],
        None,
        None
    )
    trajectory_id = len(trajectories)
    trajectories.append(trajectory)
    return {
        'trajectory_id': trajectory_id
    }

def query(args):
    trajectory_id = args['trajectory_id']
    time = args['time']
    trajectory = trajectories[trajectory_id]
    return {
        # reparameterize by arc length instead?
        'position': trajectory.get_position(time)
    }

def handle_request(request):
    handlers = {
        'generate': generate,
        'query': query
    }
    command = request['command']
    args = request['args']
    response = handlers[command](args)
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
