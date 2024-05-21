from flask import Flask, render_template, request, jsonify
import folium

app = Flask(__name__)

# Initial coordinates for the markers
marker_coords = [
    [52.290870, 4.936511],  # Buoy 1
    [52.290508, 4.929827],  # Buoy 2
    [52.290133, 4.930286]   # Buoy 3
]

ouderkerkerplas_center =[52.290244, 4.931136]

@app.route('/')
def index():
    # Create a map centered around the first marker's coordinates
    mymap = folium.Map(location=ouderkerkerplas_center, zoom_start=17)

    # Add markers and labels to the map
    for idx, coords in enumerate(marker_coords):
        folium.Marker(
            location=coords,
            icon=folium.Icon(icon="cloud"),
        ).add_to(mymap)
        folium.map.Marker(
            [coords[0], coords[1]],
            icon=folium.DivIcon(
                html=f"""<div style="font-size: 12pt; color : black">{f'Buoy {idx + 1}'}</div>"""
            )
        ).add_to(mymap)

    # Save the map to an HTML file
    mymap.save('templates/map.html')
    return render_template('map.html')

@app.route('/update_marker', methods=['POST'])
def update_marker():
    global marker_coords
    data = request.get_json()
    marker_id = data['id']
    marker_coords[marker_id] = [data['lat'], data['lng']]
    return jsonify(success=True)

if __name__ == '__main__':
    app.run(debug=True)