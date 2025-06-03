import folium
import math
import webbrowser
import os

def create_map(head, port, starboard, wind_dir=190):
    m = folium.Map(location=[52.2900, 4.9300], zoom_start=16)

    folium.Marker(head, popup='Head').add_to(m)
    folium.Marker(port, popup='Port').add_to(m)
    folium.Marker(starboard, popup='Starboard').add_to(m)

    midpoint = (
        (head[0] + port[0] + starboard[0]) / 3,
        (head[1] + port[1] + starboard[1]) / 3
    )

    add_wind_arrow(m, midpoint, wind_dir)

    # Save map and add meta-refresh
    filepath = "live_map.html"
    m.save(filepath)

    with open(filepath, "r+") as f:
        content = f.read()
        f.seek(0)
        if "<head>" in content:
            content = content.replace(
                "<head>",
                '<head>\n<meta http-equiv="refresh" content="5">\n'
            )
        f.write(content)
        f.truncate()

    return filepath

def add_wind_arrow(map_obj, location, direction, length=0.0005, arrow_length=0.0002):
    rad = math.radians(direction)
    lat, lon = location
    delta_lat = length * math.cos(rad)
    delta_lon = length * math.sin(rad)
    arrow_end = (lat + delta_lat, lon + delta_lon)

    folium.PolyLine([location, arrow_end], color='blue', weight=5).add_to(map_obj)

    left_rad = math.radians(direction + 135)
    right_rad = math.radians(direction - 135)

    left_end = (arrow_end[0] + arrow_length * math.cos(left_rad),
                arrow_end[1] + arrow_length * math.sin(left_rad))
    right_end = (arrow_end[0] + arrow_length * math.cos(right_rad),
                 arrow_end[1] + arrow_length * math.sin(right_rad))

    folium.PolyLine([arrow_end, left_end], color='blue', weight=5).add_to(map_obj)
    folium.PolyLine([arrow_end, right_end], color='blue', weight=5).add_to(map_obj)

# Example of continuously updating
if __name__ == "__main__":
    import time

    # Open once
    path = create_map(
        (52.28926899869115, 4.926260253375628),
        (52.28969490726184, 4.934093649446976),
        (52.29136300973761, 4.932715551989979)
    )
    webbrowser.open("file://" + os.path.abspath(path))

    # Simulate live updates
    for i in range(100, 10000):
        head = (52.289268 + i * 0.00001, 4.926260)
        port = (52.289694, 4.934093 - i * 0.00001)
        starboard = (52.291363, 4.932715 + i * 0.00001)
        create_map(head, port, starboard)
        time.sleep(5)
