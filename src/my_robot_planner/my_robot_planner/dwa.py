# import numpy as np

# class DWAPlanner:
#     def __init__(self, max_speed, max_accel, dt):
#         self.max_speed = max_speed
#         self.max_accel = max_accel
#         self.dt = dt

#     def calculate_trajectory(self, current_vel, goal_vel):
#         # Sample velocities
#         velocities = np.linspace(-self.max_speed, self.max_speed, 10)
#         trajectories = []
#         for v in velocities:
#             trajectory = v * self.dt  # Simplified trajectory
#             trajectories.append(trajectory)
#         return trajectories

#     def evaluate_trajectory(self, trajectories, goal):
#         # Evaluate based on distance to goal and obstacles
#         scores = []
#         for traj in trajectories:
#             score = -np.linalg.norm(goal - traj)
#             scores.append(score)
#         return trajectories[np.argmax(scores)]
# scripts/dwa_planner.py
import numpy as np

class DWAPlanner:
    def __init__(self, config):
        self.config = config

    def plan(self, x, goal, obstacles):
        dw = self.calc_dynamic_window(x)
        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, obstacles)
        return u, trajectory

    def calc_dynamic_window(self, x):
        Vs = [self.config.min_speed, self.config.max_speed, -self.config.max_yawrate, self.config.max_yawrate]
        Vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_dyawrate * self.config.dt,
              x[4] + self.config.max_dyawrate * self.config.dt]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, x, dw, goal, obstacles):
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array(x)
        for v in np.arange(dw[0], dw[1], self.config.v_reso):
            for y in np.arange(dw[2], dw[3], self.config.yawrate_reso):
                trajectory = self.predict_trajectory(x_init, v, y)
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.calc_obstacle_cost(trajectory, obstacles)
                final_cost = to_goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
        return best_u, best_trajectory

    def predict_trajectory(self, x_init, v, y):
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, y])
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt
        return trajectory

    def motion(self, x, u):
        x[0] += u[0] * np.cos(x[2]) * self.config.dt
        x[1] += u[0] * np.sin(x[2]) * self.config.dt
        x[2] += u[1] * self.config.dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_to_goal_cost(self, trajectory, goal):
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        return self.config.to_goal_cost_gain * np.hypot(dx, dy)

    def calc_obstacle_cost(self, trajectory, obstacles):
        ox = obstacles[:, 0]
        oy = obstacles[:, 1]
        dx = trajectory[:, 0][:, None] - ox[None, :]
        dy = trajectory[:, 1][:, None] - oy[None, :]
        r = np.hypot(dx, dy)
        if np.array(r <= self.config.robot_radius).any():
            return float("Inf")
        return self.config.obstacle_cost_gain * np.min(r)

class Config:
    def __init__(self):
        self.max_speed = 1.0
        self.min_speed = -0.5
        self.max_yawrate = 40.0 * np.pi / 180.0
        self.max_accel = 0.2
        self.max_dyawrate = 40.0 * np.pi / 180.0
        self.v_reso = 0.01
        self.yawrate_reso = 0.1 * np.pi / 180.0
        self.dt = 0.1
        self.predict_time = 3.0
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_radius = 1.0

if __name__ == '__main__':
    config = Config()
    dwa = DWAPlanner(config)
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    goal = np.array([10.0, 10.0])
    obstacles = np.array([[5.0, 5.0], [3.0, 6.0], [3.0, 8.0], [3.0, 10.0], [7.0, 5.0], [9.0, 5.0]])
    u, trajectory = dwa.plan(x, goal, obstacles)
    print("Best control: ", u)
    print("Trajectory: ", trajectory)