import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from missileSimulation import runSimulation

# set up the app with wide view preset and a title
st.set_page_config(layout="wide")
st.title("2D Missile Simulator Environment")
st.markdown(
    "This is an example simulation of a 2D missile engagment. The engagement is configurable with initial conditions and a navigation gain. " 
    "This navigation gain is used in the proportional navigation guidance law used by the missile. "
    "The time listed is the time of flight of the missle. "
    "Miss distance is the final point of closest approach. "
    "Score is evaluated as 100-(miss distance)+(closing velocity)."
)
metric_list = list(
    (
        "X Position",
        "Y Position",
        "X Velocity",
        "Y Velocity",
        "X Acceleration",
        "Y Acceleration",
        "Speed",
        "Lateral Acceleration",
        "Thrust",
    )
)

metric_labels = dict(
    {
        "X": "X Position",
        "Y": "Y Position",
        "dX": "X Velocity",
        "dY": "Y Velocity",
        "ddX": "X Acceleration",
        "ddY": "Y Acceleration",
        "vel": "Speed",
        "latAcc": "Lateral Acceleration",
        "thrust": "Thrust",
    }
)

# function to be used in widget argument format_func that maps metric values to readable labels, using dict above
def format_metric(metric_raw):
    return metric_labels[metric_raw]


# put all widgets in sidebar and have a subtitle
with st.sidebar:
    st.subheader("Configure the plot")
    initialMslX = st.text_input(
        "Missile Initial X Position", value="0", max_chars=5, type="default"
    )
    initialMslY = st.text_input(
        "Missile Initial Y Position", value="0", max_chars=5, type="default"
    )
    initialTgtX = st.text_input(
        "Target Initial X Position", value="-100", max_chars=5, type="default"
    )
    initialTgtY = st.text_input(
        "Target Initial Y Position", value="500", max_chars=5, type="default"
    )
    navGain = st.slider(
        label="Navigation Constant", min_value=1.0, max_value=7.0, value=3.0, step=0.1
    )
    metrics = st.multiselect(
        label="Which parameters should be plotted?",
        options=metric_labels.keys(),
        format_func=format_metric,
    )


def plotVarVsTime(figure, time, mslVar, tgtVar, ylabel):
    figure.add_trace(go.Scatter(x=time, y=tgtVar, name="Target"))
    figure.add_trace(go.Scatter(x=time, y=mslVar, name="Missile"))
    figure.update_layout(xaxis_title="Time (s)", yaxis_title=ylabel)


# Run and evaluate simulation given the initial conditions
TargetDf, MissileDf, score, missDist = runSimulation(
    initialTgtX, initialTgtY, initialMslX, initialMslY, navGain
)

# create the figure
fig = go.Figure()

# Add traces
fig.add_trace(go.Scatter(x=TargetDf["X"], y=TargetDf["Y"], mode="lines", name="Target"))
fig.add_trace(
    go.Scatter(x=MissileDf["X"], y=MissileDf["Y"], mode="lines", name="Missile")
)
fig.update_layout(xaxis_title="X", yaxis_title="Y")

fig.update_yaxes(scaleanchor="x", scaleratio=1)

st.markdown("**Simulation Statistics**")
st.markdown(f"Final Time: {MissileDf['time'].iloc[-1]:.3f}")
st.markdown(f"Miss Distance: {missDist:.3f}")
st.markdown(f"Score: {score:.3f}")

# display the plot
st.plotly_chart(fig, use_container_width=True)

for metric in metrics:
    figXPos = go.Figure()
    plotVarVsTime(
        figXPos,
        MissileDf["time"],
        MissileDf[metric],
        TargetDf[metric],
        metric_labels[metric],
    )
    st.plotly_chart(figXPos, use_container_width=True)
