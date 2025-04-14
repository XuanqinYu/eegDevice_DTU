import asyncio
import numpy as np
from bleak import BleakClient, BleakScanner
import struct
import datetime

#timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
timestamp = datetime.datetime.now().strftime("%Y%m%d")
filename = f"EEG_data_{timestamp}.txt"
notify_time = 3 # unit: seconds
RETRY_INTERVAL = 1   # fialed to find device, wait 1 s to restart


DEVICE_NAME = "EEG_BLE"  # optinal
DEVICE_ADDRESS = "20:43:A8:6E:37:DA" #4-channel borad 1
#DEVICE_ADDRESS = "CC:DB:A7:98:A6:2A" # ESP32 dev. board
#DEVICE_ADDRESS = "20:43:A8:6F:75:EA"
CHARACTERISTIC_UUID = "00002afe-0000-1000-8000-00805f9b34fb"   # data
ADS_MODE_UUID = "00002a57-0000-1000-8000-00805f9b34fb"  # ads mode

ads_mode = 1 # mode x for x channel, x belongs {1, 2, 3, 4}. ULP should use 1
async def notification_handler(sender, rdata):
    global data
    uint16_values = list(struct.unpack("<" + "H" * (len(rdata) // 2), rdata))
    with open(filename, "a") as file:
        a = 0
        for voltage in uint16_values:
            if voltage >= 65500:
                voltage =0
            
            voltage = int(voltage)/32767*3.3/0.8  # mode: five quater
            
            file.write(f"{voltage}\n")
            #print(voltage)
            a+=1
    print(f"Data Received Over {a}")

async def read_ble_data(device_address):
    async with BleakClient(device_address) as client:
        #if await client.is_connected():
        print(f"Connected to {device_address}")
        
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Subscribed to notifications. Press Ctrl+C to exit.")
        
        # send ads_mode
        await client.write_gatt_char(ADS_MODE_UUID, struct.pack('B', ads_mode))
        print(f"Written ads_mode {ads_mode} to {ADS_MODE_UUID}")
        
        await asyncio.sleep(notify_time)  # notification
        print("Notify", notify_time, "seconds")
        #await client.stop_notify(CHARACTERISTIC_UUID)
        return True
        #else:
        #    print("Failed to connect.")
        #    return False
'''
async def scan_for_device():
    while True:
        print("🔍 searching BLE device...")
        devices = await BleakScanner.discover()
        for device in devices:
            if DEVICE_NAME and device.name == DEVICE_NAME:
                print(f"✅ Find target: {device.address} ({device.name})")
                return device.address
            elif not DEVICE_NAME:
                print(f"🎯 Find target: {device.address} ({device.name})")
                return device.address

        print(f"⚠️ Failed to find target，retry after {RETRY_INTERVAL} seconds...")
        await asyncio.sleep(RETRY_INTERVAL)

        
async def scan_for_device():

    while True:
        scanner = BleakScanner()
        await scanner.start()
        print("🔍 searching BLE device...")
        for _ in range(10):  
            devices = await scanner.get_discovered_devices()
            for device in devices:
                if DEVICE_NAME and device.name == DEVICE_NAME:
                    await scanner.stop()
                    print(f"✅ find target: {device.address} ({device.name})")
                    return device.address
            await asyncio.sleep(0.2)  

        await scanner.stop()
        print("⚠️ Failed to find target")
    return None
'''
async def scan_for_device():
    scanner = BleakScanner()
    scanner.set_scanning_filter(
        duplicates=True,  
        rssi_threshold=-70  
        )

    while True:
        print("🔍 searching BLE device...")
        device = await scanner.find_device_by_address(DEVICE_ADDRESS)
        
        if device:
            print(f"✅ Find target: {DEVICE_ADDRESS}")
            return DEVICE_ADDRESS
        else:
            print("⚠️ Failed to find target")
            
        await asyncio.sleep(0.2)  # 200ms 



#asyncio.run(read_ble_data())
async def main():
    global DEVICE_ADDRESS
    while True:
        DEVICE_ADDRESS = await scan_for_device()
        if DEVICE_ADDRESS:
            success = await read_ble_data(DEVICE_ADDRESS)
            if not success:
                print("🔄 Restart the searching...")
        else:
            await asyncio.sleep(RETRY_INTERVAL)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())


