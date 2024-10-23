import webview

# URL of the HTML file to display
url = 'file:///C:/tmp/NicE-Buoy/Firmware/RoboPython/openstreetmap_with_wind_arrow.html'

# Create a window to display the HTML content
window = webview.create_window('OpenStreetMap Display', url, width=800, height=600)

# Start the application
webview.start()
