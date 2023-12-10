import threading

from read_sensors import Read_Sensors
from game_simonsays import SimonSays
from game_danceparty import DanceParty
from game_guessanimal import GuessAnimal

class Games:

    def __init__(self):
        self.game = 0

        self.dance = DanceParty()
        self.guess = GuessAnimal()
        self.simon = SimonSays()

        self.read = Read_Sensors()

    def user_input(self):
        self.game = input("Do you want to play danceparty (1), guess the animal (2), simon says (3) or exit (4)?")


    async def GameMenu(self, client):
        #input 2 for phase 2

        while self.game != "4":

            InputThread = threading.Thread(target=self.user_input)
            InputThread.start()

            await self.read.Read_char(client, 2)

            if self.game == "1" or self.game == "2" or self.game == "3":
                if self.game == "1":
                    self.dance.play()
                elif self.game == "2":
                    self.guess.play()
                elif self.game == "3":
                    self.simon.loop()

        return

        
    