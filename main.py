from simulation import Simulation
import matplotlib.pyplot as plt
import argparse
import sys, os

CONTROLLERS = ["Pure Pursuit", "Stanley", "Stanley with lookahead", "Hybrid Stanley and Pure Pursuit"]

def write_data(array, times, filename, controllers):

    file = open(filename, "wt+")
    print(*times)
    print(*(["Time"]+controllers), sep=',', file=file)
    for i in range(len(times[0])-1):
        print(*[times[0][i], array[0][i],array[1][i],array[2][i],array[3][i]], sep=',', file=file)
    file.close()



waypoint_file = "tracks/test_track_waypoints.txt"
def main(waypoint_file: str = waypoint_file, speed: int = 50, time: int=20, show: bool=False, controller: str=None):

    if controller is not None:
        test_controllers = [controller]
    else:
        test_controllers = CONTROLLERS
    ce_err = []
    rce_err = []
    yaw = []
    yaw_change = []
    total_ce = []
    total_rce = []
    times = []
    for control in test_controllers:
        print(control)
        sim = Simulation(control, control, speed, waypoint_file, time, False)
        results = sim.run()
        ce_err.append(results[0])
        rce_err.append(results[1])
        yaw.append(results[2])
        total_ce.append(results[3])
        total_rce.append(results[4])
        times.append(results[5])
        yaw_change.append(results[6])
    print("times:", times)
    print("yaw_change:", yaw_change)
    print("total_ce:", total_ce)
    if show:
        ce_fig = plt.figure(figsize=(4,4))
        ce_ax = ce_fig.add_subplot(111)
        for i in range(len(test_controllers)):
            ce_ax.plot(times[i], ce_err[i], label=test_controllers[i].replace(" ", "\n"))

        ce_ax.set_title("Crosstrack error")
        ce_ax.set_xlabel("Time $s$")
        ce_ax.set_ylabel("Crosstrack error $m$")
        plt.legend()

        total_ce_fig = plt.figure(figsize=(4,4))
        total_ce_ax = total_ce_fig.add_subplot(111)
        total_ce_ax.bar([controller.replace(" ", "\n") for controller in test_controllers], total_ce)
        total_ce_ax.set_title("Total Front Crosstrack Error")
        total_ce_ax.set_xlabel("Controller")
        total_ce_ax.set_ylabel("Crosstrack error $m$")


        rce_fig = plt.figure(figsize=(4,4))
        rce_ax = rce_fig.add_subplot(111)
        for i in range(len(test_controllers)):
            rce_ax.plot(times[i], rce_err[i], label=test_controllers[i].replace(" ", "\n"))

        rce_ax.set_title("Rear Crosstrack error")
        rce_ax.set_xlabel("Time $s$")
        rce_ax.set_ylabel("Rear Crosstrack error $m$")
        plt.legend()
        total_rce_fig = plt.figure(figsize=(4,4))
        total_rce_ax = total_rce_fig.add_subplot(111)
        total_rce_ax.bar([controller.replace(" ", "\n") for controller in test_controllers], total_rce)
        print("Total rear error:", total_rce)
        print("Total error:", total_ce)
        total_rce_ax.set_title("Total Rear Crosstrack Error")
        total_rce_ax.set_xlabel("Controller")
        total_rce_ax.set_ylabel("Rear crosstrack error $m$")
        plt.xticks(rotation=45, ha="right")



        yaw_fig = plt.figure(figsize=(4,4))
        yaw_ax = yaw_fig.add_subplot(111)
        for i in range(len(test_controllers)):
            yaw_ax.plot(times[i], yaw[i], label=test_controllers[i].replace(" ", "\n"))

        yaw_ax.set_title("Steering angle")
        yaw_ax.set_xlabel("Time $s$")
        yaw_ax.set_ylabel("Steering angle $rad$")
        plt.legend()

        yawc_fig = plt.figure(figsize=(4,4))
        yawc_ax = yawc_fig.add_subplot(111)
        for i in range(len(test_controllers)):
            yawc_ax.plot(times[i], yaw_change[i], label=test_controllers[i].replace(" ", "\n"))

        yawc_ax.set_title("Steering angle change at 50 km/h")
        yawc_ax.set_xlabel("Time $s$")
        yawc_ax.set_ylabel("Steering angle change $rad$")

        plt.legend()
        plt.show()

    prefix = f"results/{waypoint_file.split('/')[-1]}_{speed}_"
    write_data(rce_err, times, f"{prefix}rear_ce.csv", test_controllers)
    write_data(yaw_change, times, f"{prefix}yaw_change.csv", test_controllers)
    write_data(ce_err, times, f"{prefix}ce_err.csv", test_controllers)
    write_data(yaw, times, f"{prefix}yaw.csv", test_controllers)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Master script for lateral control")
    parser.add_argument('-w', '--waypoint-file',metavar='WAYPOINT_FILE',dest='waypoint_file',required=False,default=waypoint_file)
    parser.add_argument('-s', '--speed', metavar='SPEED', dest='speed', required=True)
    parser.add_argument('-t', '--time', metavar='TIME', dest='time', required=True)
    parser.add_argument('-o', '--show',  dest='show', default=False, action='store_true')
    parser.add_argument(
        '-l', '--lateral-controller',
        metavar='LATERAL_CONTROLLER',
        dest='lateral_controller',
        choices=CONTROLLERS,
        help=f'Lateral controller to choose: {CONTROLLERS}',
        default=CONTROLLERS[0],
        required=False
    )
    waypoint_file: str
    speed: str
    time: str
    show: bool
    args = parser.parse_args()
    main(args.waypoint_file, int(args.speed), int(args.time), args.show, args.lateral_controller)
