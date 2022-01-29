import sys
from lateral_controllers import LateralController, waypoint_is_in_front_of_car
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interp
from scipy.spatial import geometric_slerp


class Simulation:
    WAYPOINT_ACCEPT_DIST = 8.0

    def __init__(self, name, control_opt: str, speed: int, waypoint_file_name, max_time: int, show_results = True):
        self.name = name

        self.xc = 0
        self.yc = 0
        self.fx= 0
        self.fy = 0
        self.rx = 0 # Rear wheel x-coordinate
        self.ry = 0 # Rear wheel y-coordinate
        self.theta = np.pi
        self.delta = 0
        self.beta = 0
        self.v = speed / 3.6  # 13.3 m/s, 30 mph
        self.show_results = show_results

        self.L = 10
        self.lr = 1.2
        self.L2 = 10
        self.w_max = np.deg2rad(45.0)

        self.k_dd = 0.9
        self.min_ld = 4.0
        self.MIN_DIST_TO_END = 5.0
        self.stanley_k = 2.0
        self.lookahead_stanley_k = 0.2
        self.stanley_acceptable_bounds = 0.5
        self.lookbehind_stanley_k = 0.5
        self.ls_prev = 0

        controller_opts = {"Pure Pursuit": self.perform_pure_pursuit_control,
                           "Stanley": self.perform_stanley_control,
                           "Stanley with lookahead": self.perform_lookahead_stanley,
                           "Hybrid Stanley and Pure Pursuit": self.stanley_control_and_pp,
                           "lookbehind_stanley": self.perform_lookbehind_stanley}
        self.controller = controller_opts[control_opt]

        self.crosstrack_error_history = []
        self.yaw_history = []
        self.x_history = []
        self.y_history = []
        self.yaw_history = []
        self.time_hist = []
        self.yaw_change_hist = []
        self.rear_error_hist = []
        self.prev_w = 0
        self.timestep_factor = 10
        self.times = np.arange(0, max_time, 1/self.timestep_factor)
        self.timestep = 0
        self.timestamp = time.time()

        self.prev_ind = 0

        waypoint_file = open(waypoint_file_name)
        self.interp_range = 0.05
        self.waypoints = []
        for line in waypoint_file:
            if line.startswith("#"): continue 
            line = line.replace('\n', '')

            parts = line.split("=")
            x = []
            y = []
            dist = 0

            for i in range(1, len(parts) - 1):
                p1 = [float(p) for p in parts[i].split(",")]
                p2 = [float(p) for p in parts[i + 1].split(",")]
                dist += np.hypot(p1[0] - p2[0], p1[1] - p2[1])
                if i == 1:
                    x.append(p1[0])

                    y.append(p1[1])
                x.append(p2[0])
                y.append(p2[1])
            print(line, x, y)
            n_interps = int(dist / self.interp_range)
            new_x = np.linspace(x[0], x[-1], n_interps)
            new_y = []

            if x[-1] - x[0] == 0:
                cnt = 0
                for i in range(n_interps):
                    new_y.append(y[0] + (i * ((y[-1] - y[0]) / n_interps)))
                    cnt += self.interp_range / n_interps
            else:
                if parts[0] == "l":
                    f = interp.interp1d(x, y)
                    new_y = f(new_x)
                elif parts[0].startswith("c"):
                    p1 = parts[0].split(":")
                    centre = np.array([float(x) for x in p1[1].split(",")])
                    start = np.array([x[0], y[0]])
                    end = np.array([x[1], y[1]])
                    radius = np.hypot((start - centre)[0], (start - centre)[1])
                    p1, p2 = (start - centre) / radius, (end - centre) / radius
                    t_vals = np.linspace(0, 1, 20)
                    try:
                        result = geometric_slerp(p1, p2, t_vals)
                        new_x, new_y = result[..., 0], result[..., 1]
                        for i in range(len(new_x)):
                            new_x[i] *= radius
                            new_x[i] += centre[0]
                            new_y[i] *= radius
                            new_y[i] += centre[1]
                    except ValueError:
                        print(f"Not a sphere: start {start} centre: {centre} end: {end}")
                        raise
            for i in range(len(new_x)):
                self.waypoints.append([new_x[i], new_y[i]])
        plt.ion()
        self.fig = plt.figure(figsize=(8, 6))
        self.axis = self.fig.add_subplot(111)
        self.axis.set_title(f"Simulation: {control_opt}")
        self.axis.set_aspect('equal')
        self.theta = -np.pi / 2

        self.xc = self.waypoints[0][0]
        self.yc = self.waypoints[0][1]
        self.fx = self.xc
        self.fy = self.yc + self.L
        #        cx = np.arange(0, 50, 0.1)
        #        cy = [np.sin(x/5.0) * x/2.0 for x in cx]
        #        self.waypoints = [[cx[i], cy[i]] for i in range(len(cx))]
        self.end_position = self.waypoints[-1]

    def run(self):
        start = time.time()
        for t in self.times:
            print("Current timestamp:", t)
            self.axis.clear()
            w = self.controller()
            if w > self.w_max:
                w = self.w_max
            elif w < -self.w_max:
                w = -self.w_max
            end = time.time()
            delta_time = end - start
            self.update_position(1 / self.timestep_factor, w)
            print(delta_time)
            if np.hypot(self.end_position[0] - self.xc,
                        self.end_position[1] - self.yc) < self.MIN_DIST_TO_END and self.prev_ind > len(
                    self.waypoints) - 10:

                plt.close(self.fig)
                plt.ioff()
                if self.show_results:
                    self.finish_up()
                plt.close(self.fig)
                if self.timestep < len(self.times):
                    for i in range(len(self.times) - self.timestep):
                        self.crosstrack_error_history.append(self.crosstrack_error_history[-1])
                        self.rear_error_hist.append(self.rear_error_hist[-1])
                        self.yaw_history.append(self.yaw_history[-1])
                        self.yaw_change_hist.append(self.yaw_change_hist[-1])
                return [self.crosstrack_error_history, self.rear_error_hist, self.yaw_history, np.sum(self.rear_error_hist), np.sum(self.crosstrack_error_history), self.times, self.yaw_change_hist]
            start = time.time()
            self.update_stats(w)

            self.axis.plot([w[0] for w in self.waypoints], [w[1] for w in self.waypoints])
            self.axis.arrow(self.xc, self.yc, self.fx - self.xc, self.fy - self.yc, width=2, shape='full', head_width=1)
            #self.axis.arrow(self.rx, self.ry, self.xc - self.rx, self.yc - self.ry, width=2, shape='full', head_width=0)
            self.axis.set_xlim(self.xc - 100, self.xc + 100)
            self.axis.set_ylim(self.yc - 100, self.yc + 100)
            self.fig.canvas.draw()
            self.timestep += 1

    def update_stats(self, w):
        wp = self.waypoints[self.get_closest_waypoint_by_front()]
        self.crosstrack_error_history.append(np.hypot(wp[0] - self.fx, wp[1] - self.fy))
        self.x_history.append(self.xc)
        self.y_history.append(self.yc)
        self.yaw_history.append(w)
        self.yaw_change_hist.append(w - self.prev_w)
        self.prev_w = w
        rear_wp = self.waypoints[self.get_closest_waypoint_to_car_by_rear()]
        self.rear_error_hist.append(np.hypot(rear_wp[0] - self.xc, rear_wp[1] - self.yc))
        self.prev_ind = self.get_closest_waypoint_by_front()



    #            print("delta:", self.delta, "x", self.xc, "y", self.yc)
    # exit(0)

    def update_position(self, delta_time, delta):


        length = np.hypot(self.fx - self.rx, self.fy - self.ry)
        #print("((length**2) - (self.L2**2 + self.L**2))/(-2*self.L2*self.L)", ((length**2) - (self.L2**2 + self.L**2))/(-2*self.L2*self.L))
        #rear_theta = np.arccos(((length**2) - (self.L2**2 + self.L**2))/(-2*self.L2*self.L))

        #print("Rear wheel angle:", np.rad2deg(rear_theta))
        #print("Rear wheel:", self.rx, self.ry)
        print("Middle wheel:", self.xc, self.yc)
        print("Front wheel:", self.fx, self.fy)

        self.beta = np.arctan((self.lr * np.tan(delta)) / self.L)
        self.theta += ((self.v * np.tan(delta)) / self.L) * delta_time
        self.theta = self.normalise_angle(self.theta)
        self.xc += (self.v * np.cos(self.theta)) * delta_time
        self.yc += (self.v * np.sin(self.theta)) * delta_time
        self.fx = self.xc + ((self.L) * np.cos(self.theta))
        self.fy = self.yc + ((self.L) * np.sin(self.theta))
        #rear_theta = np.arctan2(self.rx - self.xc, self.ry - self.yc)



        #rxv = self.v * np.sin(rear_theta)
        #ryv = self.v * np.cos(rear_theta)
        #self.rx  += rxv * delta_time
        #self.ry += ryv * delta_time

    def normalise_angle(self, angle):
        while angle > np.pi:
            angle -= np.pi * 2

        while angle < -np.pi:
            angle += np.pi * 2
        return angle

    def get_target_waypoint(self, lookahead_dist):
        ind = self.get_closest_waypoint_to_car_by_rear()
        wp = []
        for i in range(ind, len(self.waypoints)):
            if np.hypot(self.waypoints[i][0] - self.xc, self.waypoints[i][1] - self.yc) >= lookahead_dist:
                return self.waypoints[i]
        return self.waypoints[-1]

    def get_target_waypoint_front(self, lookahead_dist, prev_idx=0):
        ind = self.get_closest_waypoint_by_front(prev_idx)
        wp = []
        for i in range(ind, len(self.waypoints)):
            if np.hypot(self.waypoints[i][0] - self.fx, self.waypoints[i][1] - self.fy) >= lookahead_dist:
                return i
        return len(self.waypoints) - 1

    def get_closest_waypoint_behind(self, lookbehind_dist):
        ind = self.get_closest_waypoint_by_front()
        wp = []
        for i in range(0, ind):
            wp = self.waypoints[ind-i]
            print(wp, np.hypot(wp[0] - self.fx, wp[1] - self.fy))
            if np.hypot(wp[0] - self.xc, wp[1] - self.yc) >= lookbehind_dist:
                return ind-i
        return 0

    def get_closest_waypoint_to_car_by_rear(self):
        x = [wp[0] - self.xc for wp in self.waypoints]
        y = [wp[1] - self.yc for wp in self.waypoints]
        dist = np.hypot(x, y)
        return np.argmin(dist)

    def get_closest_waypoint_by_front(self, prev_idx=0):
        x = [wp[0] - self.fx for wp in self.waypoints]
        y = [wp[1] - self.fy for wp in self.waypoints]
        dist = np.hypot(x, y)
        idx = np.argmin(dist)
        return idx

    def interpolate_straight_line(self, startx, starty, endx, endy):
        f = interp.interp1d([startx, endx], [starty, endy])
        d = np.hypot(endx - startx, endy - starty)
        x = np.linspace(startx, endx, int(d / 0.1))
        y = f(x)

    def get_trajectory_heading(self):
        pass

    def perform_pure_pursuit_control(self):
        ld = self.k_dd * self.v + self.min_ld
        target_waypoint = self.get_target_waypoint(ld)
        self.axis.scatter(target_waypoint[0], target_waypoint[1], marker=(4, 2))
        self.axis.annotate("Target waypoint", (target_waypoint[0], target_waypoint[1]))

        ld = np.hypot(target_waypoint[0] - self.xc, target_waypoint[1] - self.yc)
        alpha = np.arctan2(target_waypoint[1] - self.yc, target_waypoint[0] - self.xc) - self.theta

        delta = np.arctan2(2 * self.L * np.sin(alpha) / ld, 1.0)

        return delta

    def perform_lookbehind_stanley(self):
        ld = self.lookbehind_stanley_k * self.v
        idx = self.get_closest_waypoint_behind(ld)
        self.axis.scatter(self.waypoints[idx][0], self.waypoints[idx][1], marker=(4,1))
        return self.stanley_control_main(idx)

    def perform_lookahead_stanley(self):
        ld = self.lookahead_stanley_k * self.v
        idx = self.get_target_waypoint_front(ld, self.ls_prev)
        self.ls_prev = idx
        return self.stanley_control_main(idx)

    def perform_stanley_control(self):
        idx = self.get_closest_waypoint_by_front()
        return self.stanley_control_main(idx)

    def stanley_control_main(self, idx):
        if idx == len(self.waypoints) - 1:
            heading = np.arctan2(self.waypoints[idx][1] - self.waypoints[idx - 1][1],
                                 self.waypoints[idx][0] - self.waypoints[idx - 1][0])
        else:
            heading = np.arctan2(self.waypoints[idx + 1][1] - self.waypoints[idx][1],
                                 self.waypoints[idx + 1][0] - self.waypoints[idx][0])
        heading = self.normalise_angle(heading)
        waypoint_pos = [[self.waypoints[idx][0] - self.xc],
                        [self.waypoints[idx][1] - self.yc]]
        theta = self.theta - (np.pi / 2)
        rotation_matrix = [[np.cos(theta), np.sin(theta)],
                           [-np.sin(theta), np.cos(theta)]]
        new_waypoint = np.matmul(rotation_matrix, waypoint_pos)
        #print(new_waypoint)

        front_axle = [-np.cos(self.theta + np.pi / 2),
                      -np.sin(self.theta + np.pi / 2)]
        crosstrack_error = np.dot([self.waypoints[idx][0] - self.fx, self.waypoints[idx][1] - self.fy], front_axle)
        delta = self.normalise_angle(heading - self.theta) + np.arctan2(self.stanley_k * -crosstrack_error, self.v)

        return delta

    def stanley_control_and_pp(self):
        idx = self.get_closest_waypoint_by_front()
        # if distance within bounds perform stanley
        # if outside of bounds do pure pursuit

        wp = self.waypoints[idx]
        if np.hypot(wp[0] - self.fx, wp[1] - self.fy) < self.stanley_acceptable_bounds:
            return self.perform_stanley_control()
        else:
            return self.perform_pure_pursuit_control()

    def finish_up(self):
        fig, ax = plt.subplots(2, 2, constrained_layout=True)
        self.time_hist = self.times
        yaw_hist = ax[0, 0]
        ce_hist = ax[0, 1]
        path_hist = ax[1, 0]
        yc_hist = ax[1, 1]
        yaw_hist.set_title("Delta history")
        yaw_hist.plot(self.time_hist, self.yaw_history)
        yaw_hist.set_xlabel("Time, $s$")
        yaw_hist.set_ylabel("Yaw angle, $rad$")
        yaw_hist.set_ylim(-self.w_max -0.1, self.w_max + 0.1)

        total_ce = np.sum(self.crosstrack_error_history)
        ce_hist.set_title("Crosstrack error")
#        ce_hist.set_ylim()
        ce_hist.plot(self.time_hist, self.crosstrack_error_history)
        ce_hist.set_xlabel("Time, $s$")
        ce_hist.set_ylabel("Crosstrack error, $m$")

        path_hist.set_title("Path")
        path_hist.plot(self.x_history, self.y_history)
        path_hist.plot([w[0] for w in self.waypoints], [w[1] for w in self.waypoints])

        yc_hist.set_xlabel("Time, $s$")
        yc_hist.set_ylabel("Rear error, $m$")
        yc_hist.set_title("Rear crosstrack error")
        yc_hist.plot(self.time_hist, self.rear_error_hist)
        total_rear_ce = np.sum(self.rear_error_hist)
        print("Total crosstrack error:", total_ce)
        print("Rear crosstrack error:", total_rear_ce)
        plt.show()


if __name__ == "__main__":
    sim = Simulation("Lateral control", "stanley", "waypoints.txt")
    sim.start()
