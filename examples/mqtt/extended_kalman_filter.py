"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

This file is edited from
https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py
"""
import sys
import pathlib
import base64
import time
import json
import paho.mqtt.client as mqtt
from collections import namedtuple

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import math
import matplotlib.pyplot as plt
import numpy as np

from plot import plot_covariance_ellipse

BROKER = "localhost"
PORT = 1883
GPS_TOPIC = "GPS_TOPIC"
INPUT_TOPIC = "U_TOPIC"

# EKF matrices

# Jacobian matrix
F = np.array([[1.0, 0, 0, 0],
              [0, 1.0, 0, 0],
              [0, 0, 1.0, 0],
              [0, 0, 0, 0]])

H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])

# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.5, 0.5]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

show_animation = True


def get_u() -> list[float]:
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def motion_model(x: list[float], u: list[float]) -> list[float]:
    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u
    return x


def jacob_f(x: list[float], u: list[float]) -> list[float]:
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def observation_model(x: list[float]) -> list[float]:
    z = H @ x
    return z


ObservationOutput = namedtuple(typename='ObservationOutput',
                               field_names=['x_true', 'gps_sensor', 'xd', 'ud'])


def observation(xTrue: list[float], xd: list[float],
                u: list[float]) -> ObservationOutput:
    # update the true state according to
    # the model and the given output
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    gps_sensor = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    # make a state prediction
    xd = motion_model(xd, ud)

    return ObservationOutput(x_true=xTrue, gps_sensor=gps_sensor,
                             xd=xd, ud=ud)


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


EKFEstimationOutput = namedtuple(typename='EKFEstimationOutput',
                                 field_names=['x_est', 'p_est'])


def ekf_estimation(xEst, PEst, z, u) -> EKFEstimationOutput:
    #  Predict the state and update the
    # process covariance matrix
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return EKFEstimationOutput(x_est=xEst, p_est=PEst)


def main():
    print(__file__ + " start!!")

    # we will be sending sensor data here:
    client = mqtt.Client()
    client.connect(BROKER, PORT, 60)

    sim_time = 0.0

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    while SIM_TIME >= sim_time:
        sim_time += DT
        u = get_u()

        observation_result = observation(xTrue, xDR, u)

        xTrue = observation_result.x_true
        z = observation_result.gps_sensor
        xDR = observation_result.xd
        ud = observation_result.ud

        # publish both the GPS sensor and the input
        z_flat = z.flatten()
        z_str = json.dumps(z_flat.tolist())
        client.publish(topic=GPS_TOPIC, payload=z_str)

        print(f"Send message={z_str} to topic={GPS_TOPIC}")

        u_str = json.dumps(u.tolist())
        client.publish(topic=INPUT_TOPIC, payload=u_str)

        # compute the estimation here
        # so that we can compare with the estimation
        # from the sensor fusion module
        ekf_estimation_result = ekf_estimation(xEst, PEst, z, ud)
        xEst = ekf_estimation_result.x_est
        pEst = ekf_estimation_result.p_est

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r")
            plot_covariance_ellipse(xEst[0, 0], xEst[1, 0], PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

        time.sleep(2.0)


if __name__ == '__main__':
    main()
