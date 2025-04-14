import asyncio
import numpy as np
from bleak import BleakClient
import struct
import datetime

#timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
timestamp = datetime.datetime.now().strftime("%Y%m%d")
filename = f"EEG_data_{timestamp}.txt"
notify_time = 1000


DEVICE_ADDRESS = "20:43:A8:6E:37:DA" #4-channel board 1
#DEVICE_ADDRESS = "CC:DB:A7:98:A6:2A" # ESP32 dev. board
#DEVICE_ADDRESS = "20:43:A8:6F:75:EA"
CHARACTERISTIC_UUID = "00002afe-0000-1000-8000-00805f9b34fb"  
ADS_MODE_UUID = "00002a57-0000-1000-8000-00805f9b34fb" 

ads_mode = 1 # mode x for x channel, x belongs {1, 2, 3, 4}. ULP should use 1
async def notification_handler(sender, rdata):
    global data
    uint16_values = list(struct.unpack("<" + "H" * (len(rdata) // 2), rdata))
    with open(filename, "a") as file:
    
        for voltage in uint16_values:
            #if voltage >= 65500:
            #    voltage =0
            voltage = int(voltage)/32767*3.3/0.8  # mode: five quater
            #voltage = int(voltage)

        
            file.write(f"{voltage}\n")  
            print(voltage)

async def read_ble_data():
    async with BleakClient(DEVICE_ADDRESS) as client:
        if await client.is_connected():
            print(f"Connected to {DEVICE_ADDRESS}")
        else:
            print("Failed to connect.")
            return
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Subscribed to notifications. Press Ctrl+C to exit.")
        
        await client.write_gatt_char(ADS_MODE_UUID, struct.pack('B', ads_mode))
        print(f"Written ads_mode {ads_mode} to {ADS_MODE_UUID}")
        
        print("notify", notify_time, "second")
        await asyncio.sleep(notify_time)  
        await client.stop_notify(CHARACTERISTIC_UUID)
        
asyncio.run(read_ble_data())



