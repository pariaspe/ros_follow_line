"""
controller_pid.py

Simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller.
It gives an output value error between desired reference input and measurement feedback
that minimizes the error value.

More information: http://en.wikipedia.org/wiki/PID_controller

***************
Example of use:
***************

p=PID(3.0,0.4,1.2)
p.setPoint(5.0)
while True:
    pid = p.update(measurement_value)

"""

__authors__ = "Pedro Arias-Perez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"


class PID:
    """
    Discrete PID control
    """

    def __init__(self, k_p=2.0, k_i=0.0, k_d=1.0,
                 derivator=0, integrator=0, integrator_max=500, integrator_min=-500):
        self.__kp = k_p
        self.__ki = k_i
        self.__kd = k_d
        self.__derivator = derivator
        self.__integrator = integrator
        self.__integrator_max = integrator_max
        self.__integrator_min = integrator_min

        self.__set_point = 0.0
        self.__error = 0.0

    def update(self, current_value: float) -> float:
        """
        Calculate PID output value for given reference input and feedback
        """

        # Error calculation
        self.__error = self.__set_point - current_value

        # Proportional error
        p_value = self.kp * self.__error

        # Derivative error, discrete: subtract instead of derivate
        d_value = self.kd * (self.__error - self.__derivator)
        self.__derivator = self.__error

        # Integrative error, discrete: addition instead of integration
        self.__integrator = self.__integrator + self.__error
        if self.__integrator > self.__integrator_max:
            self.__integrator = self.__integrator_max
        elif self.__integrator < self.__integrator_min:
            self.__integrator = self.__integrator_min
        i_value = self.__integrator * self.ki

        return p_value + i_value + d_value

    # def setIntegrator(self, integrator):
    #     self.integrator = integrator

    # def setDerivator(self, derivator):
    #     self.derivator = derivator

    @property
    def kp(self) -> float:
        """
        Proportional constant
        """
        return self.__kp

    @property
    def ki(self) -> float:
        """
        Integral constant
        """
        return self.__ki

    @property
    def kd(self) -> float:
        """
        Derivative constant
        """
        return self.__kd

    # def setKp(self, P):
    #     self.__Kp = P

    # def setKi(self, I):
    #     self.Ki = I

    # def setKd(self, D):
    #     self.Kd = D

    @property
    def setpoint(self) -> float:
        """
        PID setpoint
        """
        return self.__set_point

    @setpoint.setter
    def setpoint(self, set_point: float) -> None:
        """
        Initilize PID setpoint
        """
        self.__set_point = set_point
        self.__integrator = 0
        self.__derivator = 0

    @property
    def error(self) -> float:
        """
        Current error
        """
        return self.__error

    @property
    def derivator(self) -> float:
        """
        Discrete derivator
        """
        return self.__derivator

    @property
    def integrator(self) -> float:
        """
        Discrete integrator
        """
        return self.__integrator
