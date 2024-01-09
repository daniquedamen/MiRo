import threading
from read_sensors import Read_Sensors
from robot_action import call_action
from save_behaviour import Save

class Stories:

    def __init__(self):
        self.read = Read_Sensors()
        self.save = Save()
        self.finish = False
        self.start = 0


    def user_input(self):
        print("start:", self.start)
        if self.start == 0:
            self.start = input("Do you want to start the story (1) or exit (2)?")
            self.save.save_to_file(3, "ph3_input:", self.start)
            return

    async def story(self, client): 
        for i in range(7):
            sensor = 0
            while sensor == 0:
                if i == 1 or i == 5:
                    sensor = await self.read.Read_Air(client)
                    print("air")
                else:
                    sensor = await self.read.Read_IMU(client)
                    print("IMU")
            
            print("sensor:", sensor)
            action = "ph3_" + str(i+1)
            self.save.save_to_file(3, "next:", action)
            print(action)
            call_action(action)

        self.start = 2


    async def MagicWandStory(self, client):
        self.save.save_to_file(3, "ph3_intro")
        call_action("ph3_intro")

        while self.start != 2:
            await self.read.Read_char(client, 3)

            InputThread = threading.Thread(target=self.user_input)
            InputThread.start()

            IMU = await self.read.Read_IMU(client)
            if IMU == 1:
                self.start = "1"

            if self.start == "1":
               await self.story(client)

        return
