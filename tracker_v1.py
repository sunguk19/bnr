import requests, json, time
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=100)
#ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
#ser.open()
url = "http://weair.dreammug.com/_API/saveData.php"

try:
    while 1:
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
        params = {'tra_temp':words[5], 'tra_datetime':datetime, 'tra_humidity':words[7],'tra_lat':words[2], 'tra_lon':words[3], 'de_number':words[1], 'tra_co2':words[9], 'tra_finedust':words[11], 'tra_finedust_size':words[13]}
        res = requests.post(url=url, data=params)
        print(res.json())
	
except KeyboardInterrupt:
    ser.close()
"""
def getTime():
    timeArr = time.ctime().split("")
    result = ""
    result += convNTS(timeArr[5], 4)
    result += convNTS(convMTN(timeArr[1]), 2)
    result += convNTS(timeArr[3], 2)
    result += convNTS(str(timeArr[4]).replace(":",""),6)
    return result

def convMTN(_month):
    if(_month is None or _month==""):
        return "00"
    if(_month == "Jan"):
        return "01"
    elif(_month == "Feb"):
        return "02"
    elif(_month == "Mar"):
        return "03"
    elif(_month == "Apr"):
        return "04"
    elif(_month == "May"):
        return "05"
    elif(_month == "Jun"):
        return "06"
    elif(_month == "Jul"):
        return "07"
    elif(_month == "Aug"):
        return "08"
    elif(_month == "Sep"):
        return "09"
    elif(_month == "Oct"):
        return "10"
    elif(_month == "Nov"):
        return "11"
    elif(_month == "Dec"):
        return "12"
    return "00"

def convNTS(_str, _len):
    if(_str is None):
        _str = ""
    else:
        _str = str(_str)

    if(_len < 0):
        _len = 0

    realLen = len(_str)

    if(realLen == _len):
        return _str
    if(realLen < _len):
        gap = _len - realLen
        for i in range(0, gap):
            _str = "0" + _str
        return _str
    return _str[realLen - _len, realLen]
"""
