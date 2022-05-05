import numpy as np
import pandas as pd


# Class for each vehicle object
class vehicle:
    pos = np.zeros((2, 1))
    vel = np.zeros((2, 1))
    cartAccel = np.zeros((2, 1))
    accelMan = 0.0
    thrust = 0.0

    def __init__(self, pos, vel) -> None:
        self.pos = pos
        self.vel = vel

    def states(self):
        return [self.pos, self.vel]

    def propagate(self, dt, accel):
        # propagate states by dt with accel in body frame
        self.thrust = accel[0]
        self.accelMan = accel[1]
        self.pos += self.vel * dt
        velAng = -np.arctan2(self.vel[1], self.vel[0])
        ROT = np.array(
            [[np.cos(velAng), np.sin(velAng)], [-np.sin(velAng), np.cos(velAng)]]
        )
        self.accelCart = np.dot(ROT, accel)
        self.vel += self.accelCart * dt


def addTm(t, tgt, msl, TargetInfo, MissileInfo):
    # Target TM
    TM_Dataframe = pd.DataFrame(
        [
            {
                "time": t,  # Time
                "X": tgt.pos[0],  # X Position
                "Y": tgt.pos[1],  # Y Position
                "dX": tgt.vel[0],  # X Velocity
                "dY": tgt.vel[1],  # Y Velocity
                "ddX": tgt.cartAccel[0],  # X Acceleration
                "ddY": tgt.cartAccel[1],  # Y Acceleration
                "vel": np.linalg.norm((tgt.vel[0], tgt.vel[1])),  # Speed
                "latAcc": tgt.accelMan,  # Lateral Acceleration Command
                "thrust": tgt.thrust,  # Thrust Acceleration
            }
        ]
    )
    TargetInfo.append(TM_Dataframe)
    # Missile TM
    TM_Dataframe = pd.DataFrame(
        [
            {
                "time": t,  # Time
                "X": msl.pos[0],  # X Position
                "Y": msl.pos[1],  # Y Position
                "dX": msl.vel[0],  # X Velocity
                "dY": msl.vel[1],  # Y Velocity
                "ddX": msl.cartAccel[0],  # X Acceleration
                "ddY": msl.cartAccel[1],  # Y Acceleration
                "vel": np.linalg.norm((msl.vel[0], msl.vel[1])),  # Speed
                "latAcc": msl.accelMan,  # Lateral Acceleration Command
                "thrust": msl.thrust,  # Thrust Acceleration
            }
        ]
    )
    MissileInfo.append(TM_Dataframe)


def calculateRangeRate(tgt, msl):
    r = np.linalg.norm(tgt.pos - msl.pos)
    r_unit = (tgt.pos - msl.pos) / r
    return np.dot(r_unit, (tgt.vel - msl.vel))


def runSimulation(initialTgtX, initialTgtY, initialMslX, initialMslY, navGain):
    # Initial variables for the simulation
    t, tf = 0.0, 100.0
    dt = 0.01

    # Pull in initial values
    tgt = vehicle(
        np.array([float(initialTgtX), float(initialTgtY)]), np.array([25.0, 0.0])
    )
    unit_vec = np.array(
        [
            float(initialTgtX) - float(initialMslX),
            float(initialTgtY) - float(initialMslY),
        ]
    )
    unit_vec = unit_vec / np.linalg.norm(unit_vec)
    msl = vehicle(np.array([float(initialMslX), float(initialMslY)]), 30 * unit_vec)
    tgt_pos_prev = np.array([float(initialTgtX), float(initialTgtY)])
    msl_pos_prev = np.array([float(initialMslX), float(initialMslY)])
    r_dot = 0.0
    r_dot_prev = 0.0

    TargetInfo = []
    MissileInfo = []

    # Main simulation loop, terminate when closing velocity is positive
    while t < tf:
        t += dt

        # Missile Thrust profile
        if t < 0.25:
            thrust = 600 * t
        elif t < 0.75:
            thrust = 300
        else:
            thrust = 0
        # drag = 1/2 rho v^2 beta
        beta = 0.005
        rho = 1.12
        drag = 1 / 2 * rho * np.linalg.norm(msl.vel) ** 2 * beta
        msl_thrust = thrust - drag

        # missile guidance law
        # PN -> LOS_dot = 0
        #   a = N * V * Lambda_dot
        posRel = tgt.pos - msl.pos
        velRel = tgt.vel - msl.vel
        lambdaDot = (posRel[0] * velRel[1] - posRel[1] * velRel[0]) / np.linalg.norm(
            posRel
        ) ** 2
        N = float(navGain)
        a = N * np.linalg.norm(tgt.vel) * lambdaDot
        mslAccelLimit = 45
        aCmd = np.clip(a, -mslAccelLimit, mslAccelLimit)

        # propagate states
        tgt_pos_prev = np.copy(tgt.pos)
        tgt_vel_prev = np.copy(tgt.vel)
        msl_pos_prev = np.copy(msl.pos)
        msl_vel_prev = np.copy(msl.vel)
        a_T = 0
        accelMan = 1 * np.sin(2 * np.pi * t)
        tgt.propagate(dt, np.array([a_T, accelMan]))
        msl.propagate(dt, np.array([msl_thrust, aCmd]))

        # parameters
        r_dot_prev = r_dot
        r_dot = calculateRangeRate(tgt, msl)

        # exit if positive range rate
        if r_dot > 0.0 and t > 0.1:

            # Calculate PCA
            r_dot_temp = r_dot_prev
            t_temp = t - dt
            tgt.pos = tgt_pos_prev
            tgt.vel = tgt_vel_prev
            msl.pos = msl_pos_prev
            msl.vel = msl_vel_prev
            while r_dot_temp > 0.0:
                smallDt = dt / 10
                t_temp = +smallDt
                tgt.propagate(smallDt, np.array([a_T, accelMan]))
                msl.propagate(smallDt, np.array([msl_thrust, a]))
                r_dot_temp = calculateRangeRate(tgt, msl)
                print(r_dot_temp)
                addTm(t_temp, tgt, msl, TargetInfo, MissileInfo)
            break

        addTm(t, tgt, msl, TargetInfo, MissileInfo)

    # Evaluate Simulation Results:
    r = np.linalg.norm(tgt.pos - msl.pos)
    r_unit = (tgt.pos - msl.pos) / r
    r_dot = np.dot((tgt.vel - msl.vel), r_unit)
    missDist = np.linalg.norm(tgt.pos - msl.pos)
    score = 100 - missDist - r_dot

    # Collapse TM
    TargetDf = pd.concat(TargetInfo)
    MissileDf = pd.concat(MissileInfo)
    return TargetDf, MissileDf, score, missDist
