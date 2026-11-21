import params as P

class Battery:
    """Bus voltage sags with the current the whole robot draws."""
    def voltage(self, i_total):
        return P.V_OC - P.R_BATT * i_total
