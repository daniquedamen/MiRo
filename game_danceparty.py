
'''
Danceparty game
'''
import random
import time

from read_sensors import Read_Sensors
from movements import controller

class DanceParty:

    def __init__(self):
        self.read = Read_Sensors()

    def play(self):
            
            musictime = random.randint(200, 800)
            robottime = musictime + 100
            intro = controller("dancegame")
            x = intro.loop(0, 1000)

            time.sleep(10)

            if x == 1:
                dancegame = controller("crabsong")
                y = dancegame.loop(robottime, musictime)
                if y == 1:
                    IMU = self.read.Read_IMU
                    print(IMU)
                    if IMU == 0:
                        congratulations = controller("congratulations")
                        z = congratulations.loop(0, 1000)
                    else:
                        tryagain = controller("tryagain")
                        q = tryagain.loop(0, 1000)     
                    print("done")
                    return


if __name__=="__main__":
    DanceClass = DanceParty()
    DanceClass.play()
    print("finished")

