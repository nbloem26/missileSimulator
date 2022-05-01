import numpy as np
import pandas as pd


class vehicle():
  pos = np.zeros((2,1))
  vel = np.zeros((2,1))
  
  def __init__(self, pos, vel) -> None:
      self.pos = pos
      self.vel = vel
  
  def states(self):
    return [self.pos, self.vel]

  def propagate(self, dt, accel):
    # propagate states by dt with accel in body frame
    self.pos += self.vel*dt
    velAng = -np.arctan2(self.vel[1], self.vel[0])
    ROT = np.array([[np.cos(velAng), np.sin(velAng)], [-np.sin(velAng), np.cos(velAng)]])
    accelCart = np.dot( ROT, accel)
    self.vel += accelCart*dt


t0, t = 0.0, 0.0
tf = 100
dt = 0.01

tgt = vehicle(np.array([   0.0, 500.0]), np.array([25.0,  0.0]))
msl = vehicle(np.array([   0.0,   0.0]), np.array([ 0.0, 20.0]))
tgt_pos_prev = np.array([0.0, 0.0])
msl_pos_prev = np.array([0.0, 0.0])
r_dot = 0.0
r_dot_prev = 0.0
TM = pd.DataFrame({
    "time" : np.array([]),
    "tgt_x": np.array([]),
    "tgt_y": np.array([]),
    "msl_x": np.array([]),
    "msl_y": np.array([]),
    "tgt_dx": np.array([]),
    "tgt_dy": np.array([]),
    "msl_dx": np.array([]),
    "msl_dy": np.array([]),
})
ind = 0

while t < tf:
  t += dt

  # Missile Thrust profile
  if t < 2:
    thrust = 50*t
  elif t < 3.5:
    thrust = 100
  else:
    thrust = 0
  # drag = 1/2 rho v^2 beta
  beta = 0.0075
  rho = 1.12
  drag = 1/2*rho*np.linalg.norm(msl.vel)**2*beta
  msl_thrust = thrust - drag

  # missile guidance law
  # PN -> LOS_dot = 0
  #   a = N V_r V_msl_unit x Omega
  Tgt_pos_3d = np.append(tgt.pos, 0)
  Tgt_vel_3d = np.append(tgt.vel, 0)
  Msl_pos_3d = np.append(msl.pos, 0)
  Msl_vel_3d = np.append(msl.vel, 0)
  P_r = Tgt_pos_3d - Msl_pos_3d
  V_r = Tgt_vel_3d - Msl_vel_3d
  V_msl_unit = Msl_vel_3d / np.linalg.norm(Msl_vel_3d)
  Omega = np.cross(P_r, V_r) / np.dot(P_r, P_r)
  N = 3
  # a = -np.dot(N*np.dot(V_r, V_msl_unit), Omega)
  a = N*np.linalg.norm(V_r) * np.cross(V_msl_unit, Omega)
  a = min([a[0], 5])

  # propagate states
  tgt_pos_prev = np.copy(tgt.pos)
  tgt_vel_prev = np.copy(tgt.vel)
  msl_pos_prev = np.copy(msl.pos)
  msl_vel_prev = np.copy(msl.vel)
  a_T = 1*np.sin(2*np.pi*0.025*t)
  tgt.propagate(dt, np.array([0,a_T]))
  msl.propagate(dt, np.array([msl_thrust,a]))
  
  # parameters
  r = np.linalg.norm(tgt.pos - msl.pos)
  r_unit = (tgt.pos - msl.pos)/r
  r_dot_prev = r_dot
  r_dot = np.dot( (tgt.vel - msl.pos), r_unit)
  
  # get tm (do better)
  TM.loc[len(TM.index)] = [t, tgt.pos[0], tgt.pos[1], msl.pos[0], msl.pos[1], tgt.vel[0], tgt.vel[1], msl.vel[0], msl.vel[1]]

  ind += 1
  # exit if positive range rate
  if r_dot > 0.0:
    
    # # Calculate PCA
    r_dot_temp = r_dot_prev
    t_temp = t-dt
    tgt.pos = tgt_pos_prev
    tgt.vel = tgt_vel_prev
    msl.pos = msl_pos_prev
    msl.vel = msl_vel_prev
    while r_dot_temp > 0.0:
      t_temp =+ dt/10
      tgt.propagate(dt/10, np.array([0,a_T]))
      msl.propagate(dt/10, np.array([msl_thrust,a]))

      r = np.linalg.norm(tgt.pos - msl.pos)
      r_unit = (tgt.pos - msl.pos)/r
      r_dot_temp = np.dot( (tgt.vel - msl.pos), r_unit)
      
    print("t = ", t)
    missDist = np.linalg.norm(tgt.pos - msl.pos)
    print("miss = ", missDist )

    # score output:
    score = 100-missDist - r_dot_prev
    print('score = ', score)
    break

TM.to_csv("Data/telemetry.csv", index=False)

