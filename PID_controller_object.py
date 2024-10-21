import numpy as np


class PID_controller:
    def __init__(self):
        self.prev_action = 0  # action is in torq
        self.prev_integral = 0
        self.prev_error = 0
        

    def get_action(self, state, image_state, random_controller=False):
        # terminal, Boolean
        # timestep, int
        # x, float, [-2.4, 2.4]
        # x_dot, float, [-inf, inf]
        # theta, float, [-pi/2, pi/2], radians
        # theta_dot, float, [-inf, inf]
        # reward, int, 0 or 1
        # image state is a (800, 400, 3) numpy image array; ignore it for assignment 2

        terminal, timestep, x, x_dot, theta, theta_dot, reward = state

        if random_controller:
            return np.random.uniform(-1, 1)
        else:
            
            Kp = 1.9

           
            #Kd = 0.41
            Kd = 0.41

            
            #Ki = 0.005
            Ki = 0.005

            error = theta  # we want theta = 0 (vertical)
            dt = 0.008
            # derivative = (error - self.prev_error) / dt
            derivative = theta_dot
            integral = self.prev_integral + error * dt
            random = np.random.rand()

            # Include random disturbances
            if random <= 0.999:
                action = (Kp * error) + (Kd * derivative) + (Ki * integral)
            else:
                action = 0.003

            self.prev_action = action
            self.prev_error = error
            self.prev_integral = integral

            return action