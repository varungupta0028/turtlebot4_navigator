import numpy as np

class DWAPlanner:
    def __init__(self, max_speed, max_accel, dt):
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.dt = dt

    def calculate_trajectory(self, current_vel, goal_vel):
        # Sample velocities
        velocities = np.linspace(-self.max_speed, self.max_speed, 10)
        trajectories = []
        for v in velocities:
            trajectory = v * self.dt  # Simplified trajectory
            trajectories.append(trajectory)
        return trajectories

    def evaluate_trajectory(self, trajectories, goal):
        # Evaluate based on distance to goal and obstacles
        scores = []
        for traj in trajectories:
            score = -np.linalg.norm(goal - traj)
            scores.append(score)
        return trajectories[np.argmax(scores)]
