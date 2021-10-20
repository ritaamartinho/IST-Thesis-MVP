import sys
import requests
import json
import pandas as pd
from datetime import datetime, timedelta

def rgb_json_csv(data):
    rgb_info = pd.DataFrame(data['rgb_info'])
    rgb_info = rgb_info.rename(columns={'value': 'rgb_value'})
    all = rgb_info.join(pd.DataFrame(data['temperature']).set_index('ts'), on='ts')   
    all = all.rename(columns={'value': 'temperature'})
    all = all.join(pd.DataFrame(data['relative_humidity']).set_index('ts'), on='ts')   
    all = all.rename(columns={'value': 'relative_humidity'})
    # change unix time to normal date, add 1h because we want UTC+1 timezone, rmv ms
    all['ts'] = pd.to_datetime(all['ts']/1000,unit='s') + timedelta(hours=1)
    all['ts']= all['ts'].astype('datetime64[s]')
    return all.iloc[::-1]

def tof_json_csv(data):
    all = pd.DataFrame(data['distance'])
    all = all.rename(columns={'value': 'distance'})
    # change unix time to normal date, add 1h because we want UTC+1 timezone, rmv ms
    all['ts'] = pd.to_datetime(all['ts']/1000,unit='s') + timedelta(hours=1)
    all['ts']= all['ts'].astype('datetime64[s]')
    return all.iloc[::-1]

def ble_json_csv(data):
    state = pd.DataFrame(data['state'])
    state = state.rename(columns={'value': 'state'})
    state = state.join(pd.DataFrame(data['id']).set_index('ts'), on='ts')   
    state = state.rename(columns={'value': 'id'})
    all = state.join(pd.DataFrame(data['rssi']).set_index('ts'), on='ts')   
    all = all.rename(columns={'value': 'rssi'})
    # change unix time to normal date, add 1h because we want UTC+1 timezone, rmv ms
    all['ts'] = pd.to_datetime(all['ts']/1000,unit='s') + timedelta(hours=1)
    all['ts']= all['ts'].astype('datetime64[s]')
    return all.iloc[::-1]

def vib_noise_json_csv(data, type):
    all = pd.DataFrame(data[type])
    # change unix time to normal date, add 1h because we want UTC+1 timezone, rmv ms
    all['ts'] = pd.to_datetime(all['ts']/1000,unit='s') + timedelta(hours=1)
    all['ts']= all['ts'].astype('datetime64[s]')
    return all.iloc[::-1]

def getToken():
	username = 'youremail@xxxx.com' #email used to create your ThingsBoard account (your email associated with tennant)
	password = 'tb_password'		#password of your ThingsBoard account (or your tennant account)
    #url = 'http://iot.istartlab.tecnico.ulisboa.pt/api/auth/login'
    url = 'https://thingsboard.cloud/api/auth/login'
    headers = {'Content-Type': 'application/json', 'Accept': 'application/json'}
    loginJSON = {'username': username, 'password': password}
    tokenAuthResp = requests.post(url, headers=headers, json=loginJSON).json()
    #print(tokenAuthResp)
    token = tokenAuthResp['token']
    return token

tkn = getToken()

my_headers = {'X-Authorization':  "Bearer " + tkn, "Content-Type" : "application/json"}
#print(my_headers)
#o f6 etc etc é o uuid do device que queres os dados
url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/f60b8040-d4da-11eb-ae0e-1f8899a6f9b3/keys/timeseries"

response = requests.get(url, headers=my_headers)
#print(response) , print(response.text)

my_headers = {'X-Authorization':  "Bearer " + tkn, "Content-Type" : "application/json"}
#print(my_headers)

# start & end unix timestamp (ms)
startTS = "1625499000000"   #Monday, 5 July 2021 16:30:00 GMT+01:00
endTS = "1628071200000"     #Wednesday, 4 August 2021 11:00:00 GMT+01:00
#max number of datapoints to extract
limit = "100000000"

date_filename = "_jul5_1630_aug4_11.csv"

# Get lastest timeseries value
#url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx/values/timeseries?keys=temperature,distance,relative_hum,analog_in,rgb_info"
# Get historical timeseries data
# Get data from RGB node (get device id from ThingsBoard devices page on your account)
device_id = "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
keys = "temperature,relative_humidity,rgb_info"      #info to extract
url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/" + device_id + "/values/timeseries?keys=" + keys + "&startTs=" + startTS + "&endTs=" + endTS + "&interval=0&limit=" + limit + "&agg=NONE"
response = requests.get(url, headers=my_headers)
#print(response.text) , print(response.status_code) , print(response.json())
j = json.loads(response.text)
to_file = rgb_json_csv(j)
to_file.to_csv("rgb" + date_filename, index=False)
print("RGB DONE")


# Get data from all BLE devices 
# array with all BLE Receivers device id (get device id from ThingsBoard devices page on your account)
devices = ["xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"]
keys = "id,rssi,state"      #info to extract
cnt = 1
for dev in devices:
    url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/" + dev + "/values/timeseries?keys=" + keys + "&startTs=" + startTS + "&endTs=" + endTS + "&interval=0&limit=" + limit + "&agg=NONE"

    response = requests.get(url, headers=my_headers)
    j = json.loads(response.text)

    to_file = ble_json_csv(j)
    to_file.to_csv("ble" + str(cnt) + date_filename, index=False)
    cnt = cnt + 1
print("BLE DONE")

# Get data from all ToF devices 
# array with all ToF nodes device id (get device id from ThingsBoard devices page on your account)
devices = ["xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"]
keys = "distance"     #info to extract
cnt = 0
for dev in devices:
    url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/" + dev + "/values/timeseries?keys=" + keys + "&startTs=" + startTS + "&endTs=" + endTS + "&interval=0&limit=" + limit + "&agg=NONE"

    response = requests.get(url, headers=my_headers)
    j = json.loads(response.text)

    to_file = tof_json_csv(j)
    to_file.to_csv("tof" + str(cnt) + date_filename, index=False)
    cnt = cnt + 1
print("ToF DONE")

# Get all data from all VIB-NOISE devices 
# array with all Vibration & Noise nodes device id (get device id from ThingsBoard devices page on your account)
devices = ["xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"]
cnt = 1
for dev in devices:
    # Extract Vibration Values
    keys = "value"
    url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/" + dev + "/values/timeseries?keys=" + keys + "&startTs=" + startTS + "&endTs=" + endTS + "&interval=0&limit=" + limit + "&agg=NONE"
    response = requests.get(url, headers=my_headers)
    j = json.loads(response.text)
    to_file = vib_noise_json_csv(j,'value')
    to_file.to_csv("vib" + str(cnt) + date_filename, index=False)

    # Extract Noise Values
    keys = "mic"
    url = "https://thingsboard.cloud/api/plugins/telemetry/DEVICE/" + dev + "/values/timeseries?keys=" + keys + "&startTs=" + startTS + "&endTs=" + endTS + "&interval=0&limit=" + limit + "&agg=NONE"
    response = requests.get(url, headers=my_headers)
    j = json.loads(response.text)
    to_file = vib_noise_json_csv(j,'mic')
    to_file.to_csv("noise" + str(cnt) + date_filename, index=False)
    cnt = cnt + 1
print("VIB-NOISE DONE")