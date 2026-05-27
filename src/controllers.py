import numpy as np

class Transformations:
    """
    Mathematical reference frame transformations for Field-Oriented Control.
    """
    @staticmethod
    def clarke(ia, ib, ic):
        """Transforms abc phased currents to stationary alpha-beta frame."""
        # Using the standard power-invariant transformation matrix
        i_alpha = (2.0 / 3.0) * (ia - 0.5 * ib - 0.5 * ic)
        i_beta = (2.0 / 3.0) * ((np.sqrt(3.0) / 2.0) * ib - (np.sqrt(3.0) / 2.0) * ic)
        return i_alpha, i_beta

    @staticmethod
    def park(i_alpha, i_beta, theta_e):
        """Transforms stationary alpha-beta frame to rotating rotor d-q frame."""
        cos_t = np.cos(theta_e)
        sin_t = np.sin(theta_e)
        id_flux = i_alpha * cos_t + i_beta * sin_t
        iq_torque = -i_alpha * sin_t + i_beta * cos_t
        return id_flux, iq_torque

    @staticmethod
    def inv_park(Vd, Vq, theta_e):
        """Transforms rotating rotor d-q voltage vectors back to stationary alpha-beta."""
        cos_t = np.cos(theta_e)
        sin_t = np.sin(theta_e)
        V_alpha = Vd * cos_t - Vq * sin_t
        V_beta = Vd * sin_t + Vq * cos_t
        return V_alpha, V_beta


class PIController:
    """
    Proportional-Integral Controller with anti-windup clamping for saturation limits.
    """
    def __init__(self, Kp, Ki, limit_max, limit_min):
        self.Kp = Kp
        self.Ki = Ki
        self.limit_max = limit_max
        self.limit_min = limit_min
        self.integrator = 0.0
        self.error_old = 0.0

    def compute(self, reference, measurement, dt):
        """Calculates control effort given target tracking error."""
        error = reference - measurement
        
        # Discrete integration
        self.integrator += error * dt
        
        # Calculate proportional and integral terms
        P_term = self.Kp * error
        I_term = self.Ki * self.integrator
        
        output = P_term + I_term
        
        # --- Anti-Windup Clamping (Saturation Protection) ---
        if output > self.limit_max:
            output = self.limit_max
            self.integrator -= error * dt  # Freeze integration
        elif output < self.limit_min:
            output = self.limit_min
            self.integrator -= error * dt  # Freeze integration
            
        return output

    def reset(self):
        """Resets the history of the controller."""
        self.integrator = 0.0
        self.error_old = 0.0
