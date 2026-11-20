import params as P

class Driver:
    """BTS7960 as an averaging switch: motor sees duty x bus voltage, through a small resistance."""
    r_series = P.R_DRV

    def source_voltage(self, direction, duty, v_bus):
        return direction * duty * v_bus
