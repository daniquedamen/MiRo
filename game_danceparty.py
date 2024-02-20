
'''
Danceparty game
'''
import random

from read_sensors import Read_Sensors
from robot_action import call_action
from save_behaviour import Save

class DanceParty:

    def __init__(self):
        self.read = Read_Sensors()
        self.save = Save()


    async def play(self, client):
            
        musictime = random.randint(200, 800)

        # time for robot to stop after dancing can be further explored.
        robottime = musictime + 100
        await self.read.Read_char(client, 2)

        self.save.save_to_file(2, "intro_g1")   
        call_action("ph2_intro_g1")

        whichmusic = random.randint(1, 2)
        if whichmusic == 1:
            music = "ph2_music1"
        else: 
            music = "ph2_music2" 
        
        self.save.save_to_file(2, music)
        call_action(music, musictime, robottime)

        IMU = await self.read.Read_IMU(client)
        self.save.save_to_file(2, "IMU_activity:", IMU)

        # Now: if robot stops and child does nothing, it wins. But did it even dance?
        if IMU == 0:
            
            return 1

        return 0

