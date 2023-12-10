from bleak import BleakClient, BleakScanner
import asyncio
import uuid

class TestBluetooth:

    def __init__(self):
        self.mic_uuid = uuid.UUID("a25deb7b-ec5e-456d-b6e8-f0d0a0d49bee")
        self.imu_uuid = uuid.UUID("01410ba7-23f7-4516-8f17-25d4ebd8421c")
        self.cap_uuid = uuid.UUID("c3c8e822-f1cc-4edd-8e16-3b8624e5d377")
        self.air_uuid = uuid.UUID("659b10eb-350e-484b-b447-549ad5a258c4")


        self.device = None
        self.loop = asyncio.get_event_loop()

    async def Function(self):
        scanner = BleakScanner()

        while self.device == None:
            dev = await scanner.discover()
            for i in range (len(dev)):
                if dev[i].address == "EA:EB:62:75:2F:BE":
                    self.device = dev[i]
                    print("found IP")
                    async with BleakClient(self.device.address, loop=self.loop, timeout=100) as client:
                            while True:
                                services = await client.get_services()
                                for service in services:
                                    #print(f"Service: {service.uuid}")

                                    if service.uuid == "4bd03949-19ce-466a-9abc-c53119e26f87":
                                        #characteristics = service.characteristics
                                        #for char in characteristics:
                                            #data = await client.read_gatt_char(char.uuid)
                                            #print(f"Characteristics: {char.uuid}", data)
                                        imu_data = await client.read_gatt_char(self.imu_uuid)
                                        imu_data = int.from_bytes(imu_data, 'little')
                                        mic_data = await client.read_gatt_char(self.mic_uuid)
                                        mic_data = int.from_bytes(mic_data, 'little')
                                        cap_data = await client.read_gatt_char(self.cap_uuid)
                                        cap_data = int.from_bytes(cap_data, 'little')
                                        air_data = await client.read_gatt_char(self.air_uuid)
                                        air_data = int.from_bytes(air_data, 'little')

                                        print("imu", imu_data)
                                        print("mic", mic_data)
                                        print("cap", cap_data)
                                        print("air", air_data)


while __name__=="__main__":
    test = TestBluetooth()
    test.loop.run_until_complete(test.Function())
