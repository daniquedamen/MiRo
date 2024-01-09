# simon says  
# 7 januari 2024

import random

from read_sensors import Read_Sensors
from robot_action import call_action
from save_behaviour import Save

import time

class SimonSays:
    

    def __init__(self):
        self.read = Read_Sensors()
        self.save = Save()

        self.correct = 0

    async def Interactioncheck(self, task, client):

        read_data = {
            1: self.read.Read_Air,
            2: self.read.Read_IMU,
            3: self.read.Read_Mic,
            4: self.read.Read_CAP
        }

        for task in read_data:
            result = await read_data[task](client)
            print(result)
            if result == 1:
                return 1
            
        return 0
    

    def tasks(self):

        task_number = random.randint(1,4)

        tasks = {
            1: "ph2_squeeze",
            2: "ph2_shake",
            3: "ph2_blab",
            4: "ph2_hold"
        }

        task = tasks.get(task_number)
        self.save.save_to_file(2,"ph2_ss_task:", task)

        
        call_action(task)

        return task


    async def play(self, client):

        self.save.save_to_file(2,"ph2_ss_intro")
        call_action("ph2_intro_g3")

        await self.read.Read_char(client, 2)

        for i in range(5):
            
            task = self.tasks()

            time.sleep(5)
            interaction = await self.Interactioncheck(task, client)
            await self.read.Read_char(client, 2)

            # when the interaction is equal to the task
            if interaction == 1:
                result = "ph2_goodjob"
                print("correct")
                self.save.save_to_file(2,"ph2_ss_correct_i=:", i)
                self.correct += 1
            else:
                result = "ph2_next"
                self.save.save_to_file(2,"ph2_ss_incorrect_i=:", i)
                # action, audio)
                print("incorrect")

            call_action(result)

        if self.correct > 3:
            self.save.save_to_file(2,">3 correct")
            return 1
        
        print("simonsays loop done")
        return 0

