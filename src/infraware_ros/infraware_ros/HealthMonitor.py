#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd

class HealthMonitor:

    def __init__(self, logger):
        self.start = 0
        self.new = True
        self.t_now = 0
        self.x = np.array([1, 0], dtype=float)
        self.H = 1
        self.H_alg = 1
        self.H_dyn = 1
        self.a = 0
        self.dy = 0
        self.logger = logger
        self.health_data = []
        self.time_steps = []
        self.dm_data = []
        self.dmdt_data = []
        self.domega_data = []

        # Initial threshold values
        self.delta_low = -0.04
        self.delta_high = 0.04
        self.alg_low = 0.0
        self.alg_high = 0.5
        self.dyn_low = 0.0
        self.dyn_high = 7 * (1**-9)

        # Q-learning parameters
        self.q_table = np.zeros((10, 10, 10, 10, 10, 10, 6))  # State space and action space
        self.alpha = 0.1  # Learning rate
        self.gamma = 0.9  # Discount factor
        self.epsilon = 0.1  # Exploration rate
        self.actions = [-0.01, 0, 0.01]  # Possible changes in thresholds

        self.start = 0

    def errfunction(self, x):
        erf = 0.5 * (1 + math.erf(3 * x))
        erf = 0.5 * (1 + math.erf(6 * x - 3))
        return erf

    def calculate_angle(self, a, b, c):
        ba = a - b
        bc = c - b
        ba = ba.flatten()
        bc = bc.flatten()
        norm_ba = np.linalg.norm(ba)
        norm_bc = np.linalg.norm(bc)

        if norm_ba == 0 or norm_bc == 0:
            return 0

        cosine_angle = np.dot(ba, bc) / (norm_ba * norm_bc)
        cosine_angle = np.clip(cosine_angle, -1.0, 1.0)
        angle = np.arccos(cosine_angle)
        return angle

    def predict(self, z, flag, t):
        if self.start < 3:
            self.z___ = z
            self.z__ = z
            self.z_ = z
            self.z = z
            self.start += 1
        else:
            self.z___ = self.z__
            self.z__ = self.z_
            self.z_ = self.z
            self.z = z
        self.L = np.array([[0.9989, 0.0011],
                           [0.0011, 0.9989]], dtype=float)

        if (flag == 2) or (flag == 3):
            dt = t - self.t_now
            self.t_now = t

            dm = np.sqrt((self.z[0] - self.z_[0])**2 + (self.z[1] - self.z_[1])**2)
            self.dm_data.append(dm)
            if dm <= self.alg_low:
                self.H_alg = 0
            elif dm > self.alg_high:
                self.H_alg = 0
            else:
                self.H_alg = 1 - ((dm - self.alg_low)**2 / (self.alg_high - self.alg_low)**2)

            dm_dt = dm / dt
            self.dmdt_data.append(dm_dt)

            if dm_dt <= self.dyn_low:
                self.H_dyn = 0
            elif dm_dt > self.dyn_high:
                self.H_dyn = 0
            else:
                self.H_dyn = 1 - (dm_dt - self.dyn_low) / (self.dyn_high - self.dyn_low)

            current_angle = self.calculate_angle(self.z__, self.z_, self.z)
            previous_angle = self.calculate_angle(self.z___, self.z__, self.z_)
            angle_change = current_angle - previous_angle

            if angle_change <= self.delta_low:
                self.H_4 = 0
            elif angle_change >= self.delta_high:
                self.H_4 = 0
            elif angle_change == 0:
                self.H_4 = 1
            else:
                self.H_4 = 1 - (abs(angle_change)**2 / (self.delta_high)**2)

            self.H = min(self.H_alg, self.H_dyn, self.H_4)
            self.H = max(0, min(self.H, 1))

            self.health_data.append(self.H)
            self.time_steps.append(self.t_now)
            self.domega_data.append(angle_change)
            self.t_now += 1

            if self.H <= 0.5:
                x = 2 * self.H
                Good = 0.2 * self.errfunction(x)
                x = 1 - 2 * self.H
                Bad = 0.2 + 0.8 * self.errfunction(x)
                S = Bad + Good
                good = Good / S
                bad = Bad / S
                good = Good
                bad = 1 - Good
            else:
                x = 2 * self.H - 1
                Good = 0.2 + 0.8 * self.errfunction(x)
                x = 2 - 2 * self.H
                Bad = 0.2 * self.errfunction(x)
                S = Bad + Good
                good = Good / S
                bad = Bad / S
                good = Good
                bad = 1 - Good
            self.P = np.diag([good, bad])

            self.x = self.L.dot(self.x) / (np.sum(self.x))
            self.x = self.P.dot(self.x) / (np.sum(self.x))

            self.update_q_table(dm, dm_dt, angle_change)

    def update_q_table(self, dm, dm_dt, angle_change):
        state = (
            int((self.delta_low + 0.1) * 100),
            int((self.delta_high + 0.1) * 100),
            int(self.alg_low * 10),
            int(self.alg_high * 10),
            int(self.dyn_low * 10),
            int(self.dyn_high * 1e9),
        )
        if np.random.rand() < self.epsilon:
            action = np.random.choice(range(6))
        else:
            action = np.argmax(self.q_table[state])

        next_state = list(state)
        next_state[action // 2] += self.actions[action % 3]
        next_state = tuple(next_state)

        reward = self.H
        best_next_action = np.argmax(self.q_table[next_state])

        self.q_table[state][action] += self.alpha * (
            reward + self.gamma * self.q_table[next_state][best_next_action] - self.q_table[state][action]
        )

        self.apply_action(action)

    def apply_action(self, action):
        if action == 0:
            self.delta_low += self.actions[action % 3]
        elif action == 1:
            self.delta_high += self.actions[action % 3]
        elif action == 2:
            self.alg_low += self.actions[action % 3]
        elif action == 3:
            self.alg_high += self.actions[action % 3]
        elif action == 4:
            self.dyn_low += self.actions[action % 3]
        elif action == 5:
            self.dyn_high += self.actions[action % 3]

        # Ensure thresholds stay within reasonable bounds
        self.delta_low = max(-0.1, min(0, self.delta_low))
        self.delta_high = max(0, min(0.1, self.delta_high))
        self.alg_low = max(0, min(1, self.alg_low))
        self.alg_high = max(0, min(1, self.alg_high))
        self.dyn_low = max(0, min(1e-8, self.dyn_low))
        self.dyn_high = max(0, min(1e-8, self.dyn_high))

    def export_to_excel(self):
        if len(self.health_data) > 0:
            self.logger.info(f"Attempting to write {len(self.health_data)} entries to the file.")
            try:
                data = {
                    'Time Step': self.time_steps,
                    'Health Status': self.health_data,
                    'dm': self.dm_data,
                    'dmdt': self.dmdt_data,
                    'domega': self.domega_data,
                }
                df = pd.DataFrame(data)
                df.to_csv('health_data.txt', index=False)
                self.logger.info("Data successfully written to file.")
            except Exception as e:
                self.logger.info(f"Failed to write data to file: {e}")
        else:
            self.logger.info("No
