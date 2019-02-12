import requests, json, time
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=100)
#ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
#ser.open()
url = "http://weair.dreammug.com/_API/saveData.php"

while 1:
    try:
        words = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        response = ser.readline()
        print response
#
        now = time.localtime()
	datetime = str(now.tm_year) +"-"+ str(now.tm_mon) +"-"+  str(now.tm_mday) +"-"+  str(now.tm_hour) +"-"+  str(now.tm_min) +"-"+  str(now.tm_sec)
#
        print datetime
	fhand = response
        words = fhand.split()
        print words
        for i in range(13):
            words.append(0)
        try : 
            params = {'tra_temp':words[5], 'tra_datetime':datetime, 'tra_humidity':words[7],'tra_lat':words[2], 'tra_lon':words[3], 'de_number':words[1], 'tra_co2':words[9], 'tra_finedust':words[11], 'tra_finedust_size':words[13]}
            res = requests.post(url=url, data=params)
            print(res.json())
        except : 
            print("failed to post")
	except KeyboardInterrupt:
    ser.close()
 
