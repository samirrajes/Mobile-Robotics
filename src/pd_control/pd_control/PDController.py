class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # Calculate the error
        error = self.set_point - current_value

        # Calculate P_term (Proportional term)
        P_term = self.Kp * error

        # Calculate D_term (Derivative term)
        # No need to divide by Delta t, as it's considered in Kd tuning
        D_term = self.Kd * (error - self.previous_error)

        # Update the previous error for the next iteration
        self.previous_error = error

        # The control output is the sum of all terms
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0  # Reset previous error on set point change
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D
