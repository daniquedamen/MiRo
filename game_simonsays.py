# simon says

import random
import time

from read_sensors import Read_Sensors
from stream_audio import streamer

class SimonSays:
    

    def __init__(self):
        self.read = Read_Sensors()
        self.interaction = 0
        self.finish = 0


    def Interactioncheck(self):
        if self.read.Read_Air == 1:
            self.interaction = 1
        elif self.read.Read_IMU == 1:
            self.interaction = 2
        elif self.read.Read_Mic == 1:
            self.interaction = 3
        elif self.read.Read_CAP > 0:
            self.interaction = 4

        if self.interaction > 0:
            self.finish = 1

    def tasks(self, task):
        if task == 1:
            sound = streamer("")
        elif task == 2:
            sound = streamer("")
        elif task == 3:
            sound = streamer("")
        elif task == 4:
            sound = streamer("")
        else:
            print("invalid request simonsays")

        
        sound.loop()


    def loop(self):

        task = random.randint(1,4)
        self.tasks(task)

        while self.interaction == 0:
            self.Interactioncheck()
        # when the interaction is equal to the task
        if self.interaction == task:
            audio = streamer("goodjob")
        else:
            audio = streamer("tryagain")
            audio.loop()


        print("simonsays loop done")
