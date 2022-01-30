import argparse
from simulation import Simulation
from lateral_controllers import *

if __name__ == "__main__":

    controllers = ["Pure Pursuit", "Stanley", "Stanley with lookahead", "Hybrid Stanley and Pure Pursuit", "lookbehind_stanley"]
    argparser = argparse.ArgumentParser(description="Lateral control simulation")
    argparser.add_argument(
        '-l', '--lateral-controller',
        metavar='LATERAL_CONTROLLER',
        dest='lateral_controller',
        choices=controllers,
        required=True,
        help=f'Lateral controller to choose: {controllers}',
        default=controllers[0]
    )
    argparser.add_argument(
        '-w', '--waypoint-file',
        metavar='WAYPOINT_FILE',
        dest='waypoint_file',
        required=True,
        help=f'Waypoint file to choose',
        default='waypoints.txt'
    )
    args = argparser.parse_args()
    print(args.lateral_controller)
    controller = args.lateral_controller
    print(controller)

    sim = Simulation("Lateral control",controller,50,  args.waypoint_file, 25)
    try:
        sim.run()
    except KeyboardInterrupt:
        print("Cancelled by user")
