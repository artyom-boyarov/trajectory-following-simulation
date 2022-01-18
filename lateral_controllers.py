import numpy as np
import matplotlib.pyplot as plt


def waypoint_is_in_front_of_car(waypoint, x, y, yaw):
    normalised_coord = yaw

    if yaw < 0:
        normalised_coord = (np.pi) - yaw
    theta1 = (np.pi * 2) - normalised_coord
    theta2 = np.arctan((waypoint[0] - x) / (waypoint[1] - y))
    theta3 = (np.pi / 2) - theta2

    if theta3 < ((np.pi / 2) - theta1):
        print("At", x, y, "waypoint:", waypoint[0:2])
        print("\n****Waypoint is behind car****")
        exit(2)
        return False
    return True


# References
# https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
# http://robotics.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf

class LateralController:
    def do_lateral_control(self, waypoints, x, y, v, yaw):
        return 0


class PurePursuitController(LateralController):
    def __init__(self) -> None:
        self.k_dd = 0.4

    def do_lateral_control(self, waypoints, x, y, v, yaw):
        # Get current waypoint
        ld = self.k_dd * v
        target_waypoint = []
        for waypoint in waypoints:
            if np.hypot(waypoint[0] - x, waypoint[1] - y) >= ld and waypoint_is_in_front_of_car(waypoint, x, y,
                                                                                                     yaw):  # and waypoint[1]-y < 0:
                target_waypoint = waypoint
                break
        else:
            print("No waypoints found")
            print(ld, x, y, waypoints[0:10], waypoints[-10:-1])
            target_waypoint = waypoints[-1]
        ld = np.hypot(target_waypoint[0] - x, target_waypoint[1] - y)
        alpha_hat = np.arctan2(target_waypoint[1] - y, target_waypoint[0] - x)
        alpha = alpha_hat - yaw
        delta = np.tan(4 * np.sin(alpha) / ld)
        return delta


class StanleyController(LateralController):
    def __init__(self) -> None:
        print("Stanley controller")
        self.previous_waypoint = [0, 0]
        self.prev_heading = 0

    def do_lateral_control(self, waypoints, x, y, v, yaw):

        lowest_dist = float("inf")
        ldi = -1
        target_waypoint = [0, 0]
        for i in range(len(waypoints)):
            waypoint = waypoints[i]
            d = np.hypot(waypoint[0] - x, waypoint[1] - y)

            if d < lowest_dist:
                lowest_dist = d
                ldi = i
        print("Closest waypoint:", ldi)
        target_waypoint = waypoints[ldi]
        if ldi == len(waypoints) - 1:
            return self.prev_heading - yaw
        else:
            next_wp = waypoints[ldi + 1]
        print("Target waypoint:", target_waypoint, "next one:", next_wp)

        plt.scatter(x, y, marker=(5, 1))
        plt.annotate("Position", (x, y))
        plt.scatter(target_waypoint[0], target_waypoint[1], marker=(4, 1))
        plt.annotate("Target", (target_waypoint[0], target_waypoint[1]))
        plt.scatter(next_wp[0], next_wp[1], marker=(4, 1))
        plt.annotate("Next", (next_wp[0], next_wp[1]))

        # Calculate perpendicular point
        a, b = target_waypoint[0:2]  # x,y of target waypoint
        c, d = next_wp[0:2]  # x,y of next waypoint
        print(a, b)
        print(c, d)
        print(x, y)
        m1 = (d - b) / (c - a)
        m2 = (a - c) / (b - d)
        print("m1", m1)
        print("m2", m2)
        f = y
        e = x
        y_coord = (m1 * f - m2 * b + a - e) / (m1 - m2)
        x_coord = m1 * (y - f) + e

        plt.scatter(x_coord, y_coord, marker=(4, 1))
        plt.annotate("Perpendicular point", (x_coord, y_coord))
        plt.pause(0.001)
        plt.cla()

        crosstrack_error = np.hypot(target_waypoint[0] - x, target_waypoint[1] - y)

        waypoint_theta = np.arctan2(next_wp[1] - target_waypoint[1], next_wp[0] - target_waypoint[0])
        print("waypoint heading", np.rad2deg(waypoint_theta))
        heading_error = waypoint_theta - yaw
        print("heading error", np.rad2deg(heading_error))

        self.previous_waypoint = target_waypoint
        self.prev_heading = waypoint_theta
        return heading_error