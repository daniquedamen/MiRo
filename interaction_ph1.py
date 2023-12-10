'''
07-11-23 - Danique Damen (d.damen-1@student.utwente.nl)
This file reads the characteristic of the plushie.
If there is any action: Shaking, Squeezing, Touch or Sounds, it will receive the interaction parameter.
'''
import uuid
import asyncio
import threading 


from movements import controller
from read_sensors import Read_Sensors

# class to write requested data from characteristic to file
class ReadInteraction:

    def __init__(self):

        self.read = Read_Sensors()
        
        self.istimerrunning = 0
        self.starttime = None
        self.elapsedtime = None
        self.finish = None

        self.exit_event = threading.Event()

        self.imu = 0
        self.mic = 0
        self.cap = 0
        self.air = 0

        self.prev_mic = 0
        self.prev_imu = 0
        self.prev_cap = 0
        self.prev_air = 0

        
    def user_input(self):
        while not self.exit_event.is_set():
            self.finish = input("Enter '1' to save data from phase 1 and go to phase 2:\n")
            self.exit_event.set()
        return

    # Read interaction byte
    def action_plushie(self):
        while not self.exit_event.is_set():
            todo = None
            if (self.imu == 1):
                if (self.prev_imu == 0):
                    print("IMU changed")
                    todo = "shake"
                self.prev_imu = self.imu
            if (self.mic == 1):
                if (self.prev_mic == 0):
                    print("MIC changed")
                    todo = "speaking"
                self.prev_mic = self.mic
            if (self.cap != 0):
                if (self.prev_cap != self.cap):
                    print("CAP changed")
                    if (self.cap == 1):
                        todo = "lowest"
                    if (self.cap == 10):
                        todo = "middle"
                    if (self.cap == 100):
                        todo = "upper"
                self.prev_cap = self.cap
            if (self.air == 1):
                if (self.prev_air == 0):
                    print("AIR changed")
                    todo = "squeeze"
                self.prev_air = self.air

            if todo != None:
                action = controller(todo)
                action.loop(200, 200)
        return

    async def Readdata(self, client):
        while not self.exit_event.is_set():
            await self.read.Read_char(client, 1)
            self.imu = await self.read.Read_IMU(client)
            self.mic = await self.read.Read_IMU(client)
            self.cap = await self.read.Read_CAP(client)
            self.air = await self.read.Read_Air(client)
        return


    async def read_plushie(self, client):
        InputThread = threading.Thread(target=self.user_input)
        InputThread.start()

        ActionThread = threading.Thread(target=self.action_plushie)
        ActionThread.start()
        await self.Readdata(client)
        return





