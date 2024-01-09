import random

from read_sensors import Read_Sensors
from robot_action import call_action
from save_behaviour import Save

class GuessAnimal:

    def __init__(self):
        self.animal = 0

        self.current_animal = 0
        self.prev_animal =[0]

        self.finish = False

        self.read = Read_Sensors()
        self.save = Save()

    def whichone(self):
        # randomly pick an animal 1-5, that needs to be guessed
        self.animal = random.randint(1,5)

        tasks = {
            1: "ph2_sh_donkey",
            2: "ph2_sh_cow",
            3: "ph2_sh_dog",
            4: "ph2_sh_cat",
            5: "ph2_sh_lion"
        }

        task = tasks.get(self.animal)
        self.save.save_to_file(2, "intro:", task)
        # start instruction audio

        call_action(task)
        print("intro:", task)

    async def orderofsound(self, client):

        while not self.finish:
            self.current_animal = random.randint(1,5)
            await self.read.Read_char(client, 2)

            if self.prev_animal.count(self.current_animal) == 0:

                tasks = {
                    1: "ph2_donkey",
                    2: "ph2_cow",
                    3: "ph2_dog",
                    4: "ph2_cat",
                    5: "ph2_lion"
                }

                task = tasks.get(self.current_animal)

                self.save.save_to_file(2, "current_animal:", task)
                print("sound:", task)

                call_action(task)
                self.prev_animal.append(self.current_animal)

            if self.animal == self.current_animal:
                self.finish = True

        return
            


    async def play(self, client):

        call_action("ph2_intro_g2")
        await self.read.Read_char(client, 2)

        self.whichone()

        await self.orderofsound(client)
        self.save.save_to_file(2, "finished animals")

        interaction = await self.read.Read_IMU(client)

        if interaction == 1:
            self.save.save_to_file(2, "correct")
            print("correct") # sound
            return 1

        print("sorry try again")
        self.save.save_to_file(2, "incorrect")
        return 0