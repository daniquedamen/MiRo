'''
07-11-23 - Danique Damen (d.damen-1@student.utwente.nl)
This file reads the characteristic of the plushie.
If there is any action: Shaking, Squeezing, Touch or Sounds, it will receive the interaction parameter.
'''

import threading 

from read_sensors import Read_Sensors
from save_behaviour import Save

from robot_action import call_action

# class to write requested data from characteristic to file
class ReadInteraction:

    def __init__(self):

        self.read = Read_Sensors()
        self.save = Save()

        # continue to next phase on user input (finish) or full discovery
        self.finish = None
        # discovered already
        self.discovered = []

        # measured values
        self.imu = 0
        self.mic = 0
        self.cap = 0
        self.air = 0

        # previous measurement
        self.prev_imu = 0
        self.prev_mic = 0
        self.prev_cap = 0
        self.prev_air = 0



    def user_input(self):
        if self.finish == None:
            self.finish = input("Enter '1' to save data from phase 1 and go to phase 2:\n")
            self.save.save_to_file(1, "ph1_input:", self.finish)
            return


    # Read if there is any interaction from sensors
    async def read_sensors(self, client):

        # airp does not handle exact values but difference with prev
        self.air = await self.read.Read_Air(client)
        self.prev_air = self.action_plushie(self.air, self.prev_air, "ph1_squeeze")

        self.imu = await self.read.Read_IMU(client)
        self.prev_imu = self.action_plushie(self.imu, self.prev_imu, "ph1_shake")

        self.mic = await self.read.Read_Mic(client)
        self.prev_mic = self.action_plushie(self.mic, self.prev_mic, "ph1_talk")

        self.cap = await self.read.Read_CAP(client)
        self.prev_cap = self.action_plushie(self.cap, self.prev_cap, "ph1_hold")


    def action_plushie(self, measured, old, actionstring):
        

        if measured > old:

            if self.discovered.count(actionstring) == 0:
                old = measured
                self.discovered.append(actionstring)
            else:
                actionstring = "ph1_same"

            self.save.save_to_file(1, "ph1_action:", actionstring)
            call_action(actionstring)

        return old


    async def phase1(self, client):

        # play intro
        call_action("ph1_intro")
        self.save.save_to_file(1, "ph1_intro")

        while self.finish != "1" and len(self.discovered) != 4:
        
            await self.read.Read_char(client, 1)

            InputThread = threading.Thread(target=self.user_input)
            InputThread.start()

            await self.read_sensors(client)

        call_action("ph1_outro")
        self.save.save_to_file(1, "ph1_outro")

        
        return





