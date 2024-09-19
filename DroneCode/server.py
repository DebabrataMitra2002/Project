from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/update_location', methods=['POST'])
def update_location():
    data = request.get_json()
    lat = data.get('lat')
    lon = data.get('lon')
    print(f"Received coordinates: Latitude: {lat}, Longitude: {lon}")
    return jsonify(success=True)

if __name__ == '__main__':
    app.run(port=5000)
