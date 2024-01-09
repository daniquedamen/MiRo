import uuid
import csv
import time
from datetime import datetime


class Read_Sensors:

    def __init__(self):
        self.starttime = None
        self.elapsedtime = None


        self.mic_uuid = uuid.UUID("a25deb7b-ec5e-456d-b6e8-f0d0a0d49bee")
        self.imu_uuid = uuid.UUID("01410ba7-23f7-4516-8f17-25d4ebd8421c")
        self.cap_uuid = uuid.UUID("c3c8e822-f1cc-4edd-8e16-3b8624e5d377")
        self.air_uuid = uuid.UUID("659b10eb-350e-484b-b447-549ad5a258c4")

        self.mic = 0
        self.imu = 0
        self.cap = 0
        self.air = 0
        self.prev_air = 0

        self.phasefile = None

        self.filename = None

    async def Read_char(self, client, phase):

        # make the file only once
        if phase != self.phasefile:

            self.filename = ("/home/ddamen/mdk/mdk/bin/shared/HELPER/data/" + time.strftime("%Y%m%d_%H%M") + "phase_" + str(phase) + ".csv")

            file = open(self.filename, 'a')

            writer = csv.writer(file)

            writer.writerow(["start at:" + time.strftime("%Y%m%d_%H%M")])
            writer.writerow([])
            writer.writerow(["min", "s", "elapsed_s", "mic", "imu", "cap", "air"])
            file.close()

            self.phasefile = phase
            

        self.timer()
        elapsed_secs = int(self.elapsedtime)
        print(elapsed_secs)
        now = datetime.now()
        s = now.second
        min = now.minute


        await self.Read_Mic(client)
        await self.Read_IMU(client)
        await self.Read_CAP(client)
        await self.Read_Air(client)

        # write to file
        file = open(self.filename, 'a')
        writer = csv.writer(file)
        writer.writerow([min, s, elapsed_secs, self.mic, self.imu, self.cap, self.air])
        file.close()


    async def Read_Mic(self, client):
        self.mic = await client.read_gatt_char(char_specifier=self.mic_uuid)
        self.mic = int.from_bytes(self.mic, 'little')
        # child voice = 250-400 Hz, cry = 300-600 Hz
        if self.mic > 250:
            return 1

        return 0
    

    async def Read_IMU(self, client):
        # self set threshold 200 (= magnitude gyro)

        self.imu = await client.read_gatt_char(char_specifier=self.imu_uuid)
        self.imu = int.from_bytes(self.imu, 'little')
    
        if (self.imu > 200):
            return 1
        
        return 0
    
    async def Read_CAP(self, client):

        self.cap = await client.read_gatt_char(char_specifier=self.cap_uuid)
        self.cap = int.from_bytes(self.cap, 'little')

        if (self.cap > 0):
            return 1
        
        return 0
    
    async def Read_Air(self, client):

        self.air = await client.read_gatt_char(char_specifier=self.air_uuid)
        self.air = int.from_bytes(self.air, 'little')

        if (self.air > 100):
            return 1
        
        return 0

    def timer(self):
        if self.starttime == None:
            self.starttime = time.time() # in nanoseconds

        self.elapsedtime = (time.time() - self.starttime)


