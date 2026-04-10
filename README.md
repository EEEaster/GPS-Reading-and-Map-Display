# GPS-Reading-and-Map-Display

You need to first apply for the Baidu Map API, replace the ****************** at line 124 in the code, install the libraries below, and then you can use it after connecting the GPS device.

1. Visit Baidu Maps Open Platform(https://lbsyun.baidu.com/apiconsole/center) and register a developer account
2. Create an application, select 'Browser', and obtain an AK
3. Replace the string in the above code with your own AK

Python library requirements

# 1. Upgrade pip
python -m pip install --upgrade pip

# 2. Install the core PyQt5 library
pip install PyQt5

# 3. Install PyQt5 WebEngine
pip install PyQtWebEngine

# 4. Install the serial communication library
pip install pyserial

# 5. Install the NMEA parsing library
pip install pynmea2

python3.8 或以上（推荐 3.9 / 3.10 / 3.11）
PyQt5>=5.15.0
PyQtWebEngine>=5.15.0
pyserial>=3.5
pynmea2>=1.18.0
