# GPS-Reading-and-Map-Display With Internet

You need to first apply for the Baidu Map API, replace the ****************** at line 124 in the code, install the libraries below, and then you can use it after connecting the GPS device.

1. Visit Baidu Maps Open Platform(https://lbsyun.baidu.com/apiconsole/center) and register a developer account
2. Create an application, select 'Browser', and obtain an AK
3. Replace the string in the above code with your own AK

Python library requirements
python -m pip install --upgrade pip
pip install PyQt5
pip install PyQtWebEngine
pip install pyserial
pip install pynmea2


python3.8 and above（recommend 3.9 / 3.10 / 3.11）
PyQt5>=5.15.0
PyQtWebEngine>=5.15.0
pyserial>=3.5
pynmea2>=1.18.0


Function
1. Real-time GPS Tracking
Automatically detects serial GPS devices, supports parsing NMEA sentences like GNGGA / GNRMC / GNVTG
Supports simulating real-time GPS data stream via NMEA format text files
Displays WGS84 / BD09 dual coordinate system coordinates, number of satellites, speed, heading, and altitude in real-time
2. Location Search
Calls the Baidu Maps POI search API, supports keyword search for nearby locations
Clicking on search results can directly locate on the map and set as start or end point
3. Route Planning
Supports walking / cycling / driving navigation modes
Displays route distance, estimated time, and step-by-step directions
Key points of the planned route can be exported for review
4. Real-time Navigation Mode
Tracks current GPS location in real-time during navigation
Automatically determines if off-route and triggers re-planning (interval configurable)
Automatically prompts and exits navigation upon reaching destination
5. Historical Track Management
Route records can be saved after navigation (name, type, distance, duration, start and end points)
Historical routes can be redisplayed on the map, and support deletion and management
6. Coordinate Conversion
Built-in complete conversion chain from WGS84 → GCJ02 (Mars coordinates) → BD09 (Baidu coordinates)
All map displays automatically complete coordinate system adaptation

This system is version 1.0 and still has some bugs
