import plotly.express as px

# Sample data (you can replace this with your own data)
df = px.data.wind()

# Create the wind rose chart
fig = px.bar_polar(df, r="frequency", theta="direction", color="strength",
                   template="plotly_dark", color_discrete_sequence=px.colors.sequential.Plasma_r)

# Show the chart
fig.show()
