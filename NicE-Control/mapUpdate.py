import requests
import json

# Define the data to send
data = {
    "id": 0,  # Marker ID
    "lat": 52.290800,  # New latitude
    "lng": 4.936500  # New longitude
}

# Define the URL of your server endpoint
url = "http://localhost:5000/update_marker"  # Update the URL with your server address

# Send the POST request
response = requests.post(url, json=data)

# Check the response
if response.status_code == 200:
    print("Marker updated successfully")
else:
    print("Failed to update marker:", response.status_code)
