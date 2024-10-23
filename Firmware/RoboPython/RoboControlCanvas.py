import matplotlib.pyplot as plt
import contextily as ctx
import geopandas as gpd
from shapely.geometry import Point
import numpy as np

# Coordinates in (longitude, latitude) for plotting
head = (4.926260253375628, 52.28926899869115)
port = (4.934093649446976, 52.28969490726184)
starboard = (4.932715551989979, 52.29136300973761)

# Create a GeoDataFrame with the points
gdf = gpd.GeoDataFrame(geometry=[Point(head), Point(port), Point(starboard)], crs="EPSG:4326")

# Create a larger figure
plt.figure(figsize=(12, 8))  # Set the figure size to 12 inches wide and 8 inches tall

# Plot the points on OpenStreetMap tiles
ax = gdf.plot(marker='o', color='red', markersize=50, ax=plt.gca())  # Use the current axes

# Add OpenStreetMap as background
ctx.add_basemap(ax, crs=gdf.crs.to_string(), source=ctx.providers.OpenStreetMap.Mapnik)

# Set limits for better view
ax.set_xlim([4.92, 4.94])
ax.set_ylim([52.28, 52.30])

# Wind direction and calculation for arrow
wind_direction = 190
arrow_dx = 0.001 * np.sin(np.radians(wind_direction))
arrow_dy = 0.001 * np.cos(np.radians(wind_direction))

# Add arrow for wind direction at 'head' point
plt.arrow(head[0], head[1], arrow_dx, arrow_dy, head_width=0.0001, head_length=0.0001, fc='blue', ec='blue')

# Add titles and labels
plt.title("Map with Wind Direction and Coordinates")
plt.xlabel("Longitude")
plt.ylabel("Latitude")

# Show the plot
plt.show()
