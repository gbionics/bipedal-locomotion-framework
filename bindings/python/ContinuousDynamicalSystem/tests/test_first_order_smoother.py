import pytest

import bipedal_locomotion_framework.bindings as blf
import numpy as np

from datetime import timedelta


def test_first_order_smoother():

    dT = timedelta(microseconds=100)
    settling_time = 0.1
    updated_settling_time = 0.05
    tolerance = 1e-2

    parameters_handler = blf.parameters_handler.StdParametersHandler()
    parameters_handler.set_parameter_float("settling_time", settling_time)
    parameters_handler.set_parameter_datetime("sampling_time", dT)

    smoother = blf.continuous_dynamical_system.FirstOrderSmoother()

    # the settling time cannot be changed before the class is initialized
    assert not smoother.set_settling_time(updated_settling_time)

    assert smoother.initialize(parameters_handler)

    # the settling time cannot be changed before reset()
    assert not smoother.set_settling_time(updated_settling_time)

    initial_state = np.zeros(2)
    input = np.ones(2)

    assert smoother.reset(initial_state)
    assert smoother.set_input(input)
    assert smoother.get_settling_time() == settling_time

    # a non positive settling time is not valid
    assert not smoother.set_settling_time(0.0)
    assert not smoother.set_settling_time(-1.0)

    # let the system evolve for half of the initial settling time
    for _ in range(500):
        assert smoother.advance()

    # speed up the response by reducing the settling time. The state reached so far is used as
    # initial condition of the new dynamics
    assert smoother.set_settling_time(updated_settling_time)
    assert smoother.get_settling_time() == updated_settling_time

    state_at_switch = smoother.get_output().copy()
    a = 3.0 / updated_settling_time

    def close_form_solution(t):
        return np.ones(2) - (np.ones(2) - state_at_switch) * np.exp(-a * t)

    for i in range(1000):
        output = smoother.get_output()
        assert output == pytest.approx(close_form_solution(i * dT.total_seconds()), abs=tolerance)
        assert smoother.advance()

    # since the settling time has been reduced the system should have already converged
    assert smoother.get_output() == pytest.approx(input, abs=tolerance)
