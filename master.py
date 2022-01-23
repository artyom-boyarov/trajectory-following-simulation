from simulation import Simulation
import matplotlib.pyplot as plt
import argparse

waypoint_file = "tracks/test_track_waypoints.txt"
def main(waypoint_file: str = waypoint_file):
    controllers = ["Pure Pursuit", "Stanley", "Stanley with lookahead", "Hybrid Stanley and Pure Pursuit"]
    ce_err = []
    rce_err = []
    yaw = []
    yaw_change = []
    total_ce = []
    total_rce = []
    times = []
    for control in controllers:
        print(control)
        sim = Simulation(control, control, waypoint_file, False)
        results = sim.run()
        ce_err.append(results[0])
        rce_err.append(results[1])
        yaw.append(results[2])
        total_ce.append(results[3])
        total_rce.append(results[4])
        times.append(results[5])
        yaw_change.append(results[6])

    ce_fig = plt.figure(figsize=(4,4))
    ce_ax = ce_fig.add_subplot(111)
    for i in range(4):
        ce_ax.plot(times[i], ce_err[i], label=controllers[i])

    ce_ax.set_title("Crosstrack error")
    ce_ax.set_xlabel("Time $s$")
    ce_ax.set_ylabel("Crosstrack error $m$")
    plt.legend()

    total_ce_fig = plt.figure(figsize=(4,4))
    total_ce_ax = total_ce_fig.add_subplot(111)
    total_ce_ax.bar(controllers, total_ce)
    total_ce_ax.set_title("Total crosstrack error")
    total_ce_ax.set_xlabel("Controller")
    total_ce_ax.set_ylabel("Crosstrack error $m$")


    rce_fig = plt.figure(figsize=(4,4))
    rce_ax = rce_fig.add_subplot(111)
    for i in range(4):
        rce_ax.plot(times[i], rce_err[i], label=controllers[i])

    rce_ax.set_title("Rear Crosstrack error")
    rce_ax.set_xlabel("Time $s$")
    rce_ax.set_ylabel("Rear Crosstrack error $m$")
    plt.legend()
    total_rce_fig = plt.figure(figsize=(4,4))
    total_rce_ax = total_rce_fig.add_subplot(111)
    total_rce_ax.bar(controllers, total_rce)
    print("Total rear error:", total_rce)
    print("Total error:", total_ce)
    total_rce_ax.set_title("Total rear crosstrack error")
    total_rce_ax.set_xlabel("Controller")
    total_rce_ax.set_ylabel("Rear crosstrack error $m$")
    plt.xticks(rotation=45, ha="right")



    yaw_fig = plt.figure(figsize=(4,4))
    yaw_ax = yaw_fig.add_subplot(111)
    for i in range(4):
        yaw_ax.plot(times[i], yaw[i], label=controllers[i])

    yaw_ax.set_title("Steering angle")
    yaw_ax.set_xlabel("Time $s$")
    yaw_ax.set_ylabel("Steering angle $rad$")
    plt.legend()

    yawc_fig = plt.figure(figsize=(4,4))
    yawc_ax = yawc_fig.add_subplot(111)
    for i in range(4):
        yawc_ax.plot(times[i], yaw_change[i], label=controllers[i])

    yawc_ax.set_title("Steering angle change at 50 km/h")
    yawc_ax.set_xlabel("Time $s$")
    yawc_ax.set_ylabel("Steering angle change $rad$")


    plt.legend()
    plt.show()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Master script for lateral control")
    parser.add_argument('-w', '--waypoint-file',metavar='WAYPOINT_FILE',dest='waypoint_file',required=False,default=waypoint_file)
    args = parser.parse_args()
    main(args.waypoint_file)
