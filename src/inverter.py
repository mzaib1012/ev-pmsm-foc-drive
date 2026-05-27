import numpy as np

class SpaceVectorPWM:
    """
    Space Vector Pulse Width Modulation (SVPWM) module for a 
    3-Phase Voltage Source Inverter.
    """
    def __init__(self, V_dc):
        self.V_dc = V_dc  # High-Voltage EV Battery Pack DC Bus Link (Volts)

    def compute_gating_times(self, V_alpha, V_beta, Ts):
        """
        Determines sector location and computes the normalized dwell 
        times (T1, T2, T0) for the inverter switches.
        """
        # 1. Calculate projection components
        # Using inverse clarke structural variables for sector identification
        v1 = V_beta
        v2 = (-0.5 * V_beta) + ((np.sqrt(3.0) / 2.0) * V_alpha)
        v3 = (-0.5 * V_beta) - ((np.sqrt(3.0) / 2.0) * V_alpha)
        
        # 2. Identify Hexagon Sector (1 to 6)
        sector = 0
        if v1 >= 0:
            if v2 >= 0:
                if v3 >= 0: pass
                else: sector = 2
            else:
                if v3 >= 0: sector = 1
                else: sector = 3
        else:
            if v2 >= 0:
                if v3 >= 0: sector = 6
                else: sector = 4
            else:
                if v3 >= 0: sector = 5
                else: pass

        # Fallback security check
        if sector == 0: 
            sector = 1

        # 3. Calculate vector dwell durations based on sector math
        # Normalized by the DC Link Voltage
        const_factor = np.sqrt(3.0) * Ts / self.V_dc
        
        if sector == 1:
            T1 = const_factor * ((np.sqrt(3.0)/2.0)*V_alpha - 0.5*V_beta)
            T2 = const_factor * V_beta
        elif sector == 2:
            T1 = const_factor * ((np.sqrt(3.0)/2.0)*V_alpha + 0.5*V_beta)
            T2 = const_factor * (-((np.sqrt(3.0)/2.0)*V_alpha) + 0.5*V_beta)
        elif sector == 3:
            T1 = const_factor * V_beta
            T2 = const_factor * (-((np.sqrt(3.0)/2.0)*V_alpha) - 0.5*V_beta)
        elif sector == 4:
            T1 = const_factor * (-((np.sqrt(3.0)/2.0)*V_alpha) + 0.5*V_beta)
            T2 = const_factor * (-V_beta)
        elif sector == 5:
            T1 = const_factor * (-((np.sqrt(3.0)/2.0)*V_alpha) - 0.5*V_beta)
            T2 = const_factor * ((np.sqrt(3.0)/2.0)*V_alpha - 0.5*V_beta)
        elif sector == 6:
            T1 = const_factor * (-V_beta)
            T2 = const_factor * ((np.sqrt(3.0)/2.0)*V_alpha + 0.5*V_beta)

        # 4. Enforce saturation limits to prevent over-modulation
        if (T1 + T2) > Ts:
            normalization = Ts / (T1 + T2)
            T1 *= normalization
            T2 *= normalization
        
        T0 = Ts - T1 - T2
        
        # 5. Map timings to actual phase configurations (Symmetrical PWM generation)
        # Reconstructed average phase voltages applied back across the step
        Ta = 0.5 * T0
        Tb = Ta + T1
        Tc = Tb + T2
        
        # Sector state routing logic matrix
        if sector == 1:   t_A, t_B, t_C = Ta, Tb, Tc
        elif sector == 2: t_A, t_B, t_C = Tb, Ta, Tc
        elif sector == 3: t_A, t_B, t_C = Tc, Ta, Tb
        elif sector == 4: t_A, t_B, t_C = Tc, Tb, Ta
        elif sector == 5: t_A, t_B, t_C = Tb, Tc, Ta
        elif sector == 6: t_A, t_B, t_C = t_A, t_B, t_C = Ta, Tc, Tb
        
        # Map duty cycles back to reconstructed physical voltages for our continuous model
        V_a_eff = (t_A / Ts) * self.V_dc - (self.V_dc / 2.0)
        V_b_eff = (t_B / Ts) * self.V_dc - (self.V_dc / 2.0)
        V_c_eff = (t_C / Ts) * self.V_dc - (self.V_dc / 2.0)
        
        return V_a_eff, V_b_eff, V_c_eff, sector
