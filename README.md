# pi_nav

```bash
sudo pip3 install pynmeagps
sudo pip3 install gps3
sudo apt-get install python3-serial
sudo apt-get install gpsd gpsd-clients python3-gps
```
    
## Run
```bash
sudo python3 main.py
```
    
## Output Example

```javascript
[
  {
    "time": 1682084501.281991,
    "altitude": "n/a",
    "lon": 49.818616667,
    "lat": 24.002801667,
    "speed": 0,
    "temp": 34.6,
    "humidity": 20.8,
    "roll": 0.3756590436489526,
    "pitch": 358.87317630818563,
    "yaw": 213.11172755888998
  }]
```

## Links

 - [L76X GPS HAT API](https://www.waveshare.com/wiki/L76X_GPS_HAT)
 - [AGPS3 docs](https://github.com/wadda/gps3/blob/master/gps3/agps3.py)
 - [Use GPS module with Raspberry Pi ](https://www.dfrobot.com/blog-772.html)
 - [Use GPS module with Raspberry Pi ](https://www.dfrobot.com/blog-772.html)