import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import numpy as np

# set up the app with wide view preset and a title
st.set_page_config(layout="wide")
st.title("2D Missile Simulator Environment")
metric_list = list(('X Position', 'Y Position', 'X Velocity', 'Y Velocity', 
                    'X Acceleration', 'Y Acceleration', 'Speed', 'Lateral Acceleration', 'Thrust'))

metric_labels = dict({'X': 'X Position', 'Y': 'Y Position',
                      'dX': 'X Velocity', 'dY': 'Y Velocity', 
                      'ddX': 'X Acceleration', 'ddY': 'Y Acceleration',
                      'vel': 'Speed', 'latAcc': 'Lateral Acceleration', 'thrust': 'Thrust'})

# function to be used in widget argument format_func that maps metric values to readable labels, using dict above
def format_metric(metric_raw):
    return metric_labels[metric_raw]

# put all widgets in sidebar and have a subtitle
with st.sidebar:
    st.subheader("Configure the plot")
    initialMslX = st.text_input("Missile Initial X Position", value="0", max_chars=10, type="default")
    initialMslY = st.text_input("Missile Initial Y Position", value="0", max_chars=10, type="default")
    initialTgtX = st.text_input("Target Initial X Position", value="0", max_chars=10, type="default")
    initialTgtY = st.text_input("Target Initial Y Position", value="500", max_chars=10, type="default")
    navGain = st.slider(label = "Navigation Constant", min_value = 2.0, max_value = 7.0, value = 3.0, step=0.1)
    metrics = st.multiselect(label = "Which parameters should be plotted?", options = metric_labels.keys(), format_func=format_metric)

def plotVarVsTime(figure, time, mslVar, tgtVar, ylabel):
    figure.add_trace(go.Scatter(x=time, y=tgtVar, name='Target'))
    figure.add_trace(go.Scatter(x=time, y=mslVar, name='Missile'))
    figure.update_layout( xaxis_title="Time (s)", yaxis_title=ylabel)

# Class for each vehicle object
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
    return accelCart

# Initial variables for the simulation
t0, t = 0.0, 0.0
tf = 100
dt = 0.01

# Pull in initial values
tgt = vehicle(np.array([ float(initialTgtX), float(initialTgtY)]), np.array([25.0,  0.0]))
unit_vec = np.array([float(initialTgtX)-float(initialMslX), float(initialTgtY)-float(initialMslY)])
unit_vec = unit_vec/np.linalg.norm(unit_vec)
msl = vehicle(np.array([ float(initialMslX), float(initialMslY)]), 20*unit_vec)
tgt_pos_prev = np.array([ float(initialTgtX), float(initialTgtY)])
msl_pos_prev = np.array([ float(initialMslX), float(initialMslY)])
r_dot = 0.0
r_dot_prev = 0.0

TargetInfo = []
MissileInfo = []

# Main simulation loop, terminate when closing velocity is positive
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
  beta = 0.005
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
  N = float(navGain)
  a = N*np.linalg.norm(V_r) * np.cross(V_msl_unit, Omega)
  a = np.clip(a[0], -10.0, 10.0)

  # propagate states
  tgt_pos_prev = np.copy(tgt.pos)
  tgt_vel_prev = np.copy(tgt.vel)
  msl_pos_prev = np.copy(msl.pos)
  msl_vel_prev = np.copy(msl.vel)
  a_T = 0
  a_Man = 1*np.sin(2*np.pi*t)
  tgtCartAccel = tgt.propagate(dt, np.array([a_T,a_Man]))
  mslCartAccel = msl.propagate(dt, np.array([msl_thrust,a]))
  
  # parameters
  r = np.linalg.norm(tgt.pos - msl.pos)
  r_unit = (tgt.pos - msl.pos)/r
  r_dot_prev = r_dot
  r_dot = np.dot( (tgt.vel - msl.pos), r_unit)
  
  # get tm (do better)
  # Target TM
  TM_Dataframe = pd.DataFrame([{
      "time"   : t,
      "X"      : tgt.pos[0],
      "Y"      : tgt.pos[1],
      "dX"     : tgt.vel[0],
      "dY"     : tgt.vel[1],
      "ddX"    : tgtCartAccel[0],
      "ddY"    : tgtCartAccel[1],
      "vel"    : np.linalg.norm((tgt.vel[0], tgt.vel[1])),
      "latAcc" : a_Man,
      "thrust" : a_T,
  }])
  TargetInfo.append(TM_Dataframe)
  # Missile TM
  TM_Dataframe = pd.DataFrame([{
      "time"   : t,
      "X"      : msl.pos[0],
      "Y"      : msl.pos[1],
      "dX"     : msl.vel[0],
      "dY"     : msl.vel[1],
      "ddX"    : mslCartAccel[0],
      "ddY"    : mslCartAccel[1],
      "vel"    : np.linalg.norm((msl.vel[0], msl.vel[1])),
      "latAcc" : a,
      "thrust" : msl_thrust,
  }])
  MissileInfo.append(TM_Dataframe)

  # exit if positive range rate
  if r_dot > 0.0 and t > 2:
    
    # # Calculate PCA
    r_dot_temp = r_dot_prev
    t_temp = t-dt
    tgt.pos = tgt_pos_prev
    tgt.vel = tgt_vel_prev
    msl.pos = msl_pos_prev
    msl.vel = msl_vel_prev
    while r_dot_temp > 0.0:
      smallDt = dt/20
      t_temp =+ smallDt
      tgt.propagate(smallDt, np.array([a_T,a_Man]))
      msl.propagate(smallDt, np.array([msl_thrust,a]))

      r = np.linalg.norm(tgt.pos - msl.pos)
      r_unit = (tgt.pos - msl.pos)/r
      r_dot_temp = np.dot( (tgt.vel - msl.pos), r_unit)
      
    missDist = np.linalg.norm(tgt.pos - msl.pos)

    # score:
    score = 100-missDist - r_dot_prev
    break

# Collapse TM
TargetDf = pd.concat(TargetInfo)
MissileDf = pd.concat(MissileInfo)

# create the figure
fig = go.Figure()

# Add traces
fig.add_trace(go.Scatter( x=TargetDf['X'],  y=TargetDf['Y'], mode='lines',  name='Target'))
fig.add_trace(go.Scatter(x=MissileDf['X'], y=MissileDf['Y'], mode='lines', name='Missile'))

fig.update_xaxes( range = [-10,510] )
fig.update_yaxes(
    scaleanchor = "x",
    scaleratio = 1,
  )

st.markdown(f"Final Time: {t:.3f}")
st.markdown(f"Miss Distance: {missDist:.3f}")
st.markdown(f"Score: {score:.3f}")
  
# display the plot
st.plotly_chart(fig, use_container_width=True)

for metric in metrics:
    figXPos = go.Figure()
    plotVarVsTime(figXPos, MissileDf['time'], MissileDf[metric], TargetDf[metric], metric_labels[metric])
    st.plotly_chart(figXPos, use_container_width=True)
