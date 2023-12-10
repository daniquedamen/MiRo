import threading
from read_sensors import Read_Sensors

class Stories:

    def __init__(self):
        self.read = Read_Sensors()
        self.finish = False
        self.start = None


    def user_input(self):
        self.start = input("Do you want to start the story (1) or exit (2)?")

    async def MagicWandStory(self, client):
        while not self.finish:
            InputThread = threading.Thread(target=self.user_input)
            InputThread.start()

            await self.read.Read_char(client, 3)

            if self.start == 1:
                print("start story!")
                return
                #ActionThread = threading.Thread(target=self.function)
                #ActionThread.start()
            return