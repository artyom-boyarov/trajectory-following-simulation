import argparse
from simulation import Simulation
from lateral_controllers import *
import sys, os

if __name__ == "__main__":

    controllers = ["Pure Pursuit", "Stanley", "Stanley with lookahead", "Hybrid Stanley and Pure Pursuit"]
    argparser = argparse.ArgumentParser(description="Lateral control simulation")
    argparser.add_argument(
        '-l', '--lateral-controller',
        metavar='LATERAL_CONTROLLER',
        dest='lateral_controller',
        choices=controllers,
        help=f'Lateral controller to choose: {controllers}',
        default=controllers[0]
    )
    argparser.add_argument(
        '-w', '--waypoint-file',
        metavar='WAYPOINT_FILE',
        dest='waypoint_file',
        help=f'Waypoint file to choose',
        default='waypoints.txt'
    )
    args = argparser.parse_args()
    controller: str
    waypoint_file: str
    if len(sys.argv) < 2:
        print("Please choose control algorithm")
        print("Options:", *[f"{i:} {controllers[i]}" for i in range(len(controllers))])
        controller = controllers[int(input())]
        print("Enter waypoint file name:")
        print(f"Suggested waypoint files:", *['tracks/' + x for x in os.listdir('./tracks/')])
        waypoint_file = input()
    else:
        controller = args.lateral_controller
        waypoint_file = args.waypoint_file

    sim = Simulation("Lateral control",controller,50,  waypoint_file, 25)
    try:
        sim.run()
    except KeyboardInterrupt:
        print("Cancelled by user")
