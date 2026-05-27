import numpy as np

class PMSMMotor:
    """
    High-fidelity state-space mathematical model of an EV PMSM Traction Motor
    optimized for numeric stability in discrete solvers.
    """
    def __init__(self):
        # --- High-Power EV Traction Motor Parameters ---
        self.Rs = 0.05        # Stator resistance (Ohms)
        self.Ld = 0.0002      # d-axis inductance (Henries)
        self.Lq = 0.0002      # q-axis inductance (Henries)
        self.lmbda = 0.08     # Permanent magnet flux linkage (Webers)
        self.P = 4            # Number of pole pairs
        self.J = 0.02         # Rotor inertia (kg*m^2)
        self.B = 0.01         # Viscous friction coefficient
        
        # --- Initial State Variables ---
        self.id = 0.0         
        self.iq = 0.0         
        self.omega_m = 0.0    
        self.theta_m = 0.0    
        self.theta_e = 0.0    
        self.Te = 0.0         

    def update_states(self, Vd, Vq, TL, dt):
        """Updates electrical currents and rotational dynamics."""
        omega_e = self.omega_m * self.P
        
        # Continuous-time state derivatives (Differential equations)
        did_dt = (1.0 / self.Ld) * (Vd - self.Rs * self.id + omega_e * self.Lq * self.iq)
        diq_dt = (1.0 / self.Lq) * (Vq - self.Rs * self.iq - omega_e * self.Ld * self.id - omega_e * self.lmbda)
        
        # State integration over discrete timestep
        self.id += did_dt * dt
        self.iq += diq_dt * dt
        
        # Electromagnetic torque production
        self.Te = 1.5 * self.P * self.lmbda * self.iq
        
        # Mechanical acceleration dynamics
        domega_m_dt = (1.0 / self.J) * (self.Te - TL - self.B * self.omega_m)
        self.omega_m += domega_m_dt * dt
        
        # Rotor position calculation
        self.theta_m += self.omega_m * dt
        self.theta_m = self.theta_m % (2 * np.pi)
        self.theta_e = (self.theta_m * self.P) % (2 * np.pi)
        
        return self.id, self.iq, self.omega_m, self.theta_e, self.Te
