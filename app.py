import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

# set up the app with wide view preset and a title
st.set_page_config(layout="wide")
st.title("2D Missile Simulator Environment")

# import our data as a pandas dataframe
df = pd.read_csv("Data/telemetry.csv")

# create the figure
fig = go.Figure()

# Add traces
fig.add_trace(go.Scatter(x=df['tgt_x'], y=df['tgt_y'],
                    mode='markers',
                    name='Target'))
fig.add_trace(go.Scatter(x=df['msl_x'], y=df['msl_y'],
                    mode='lines+markers',
                    name='Missile'))

fig.update_xaxes(
    range = [-10,510]
)
fig.update_yaxes(
    scaleanchor = "x",
    scaleratio = 1,
  )
# display the plot
st.plotly_chart(fig, use_container_width=True)