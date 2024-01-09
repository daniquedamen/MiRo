import threading

from read_sensors import Read_Sensors
from game_simonsays import SimonSays
from game_danceparty import DanceParty
from game_guessanimal import GuessAnimal

from robot_action import call_action
from save_behaviour import Save

class Games:

    def __init__(self):
        self.start = 0
        self.finish = 0
        self.game = 0
        self.game_string = None

        self.dance = DanceParty()
        self.guess = GuessAnimal()
        self.simon = SimonSays()

        self.read = Read_Sensors()
        self.save = Save()

    def user_input(self):
        if self.game_string == None and self.game == 0:
            self.game_string = input("Do you want to play danceparty (1), guess the animal (2), simon says (3) or exit (4)?")

            if self.game_string == "1":
                self.game = 1
            elif self.game_string == "2":
                self.game = 2
            elif self.game_string == "3":
                self.game = 3
            elif self.game_string == "4":
                self.game = 4

            self.save.save_to_file(2, "ph2_input:", self.game)

    async def sensor_input(self, client):
        IMU = await self.read.Read_IMU(client)
        if IMU == 1:
            self.game = 1
            self.save.save_to_file(2,"ph2_IMU_game1")
        Air = await self.read.Read_Air(client)
        if Air == 1:
            self.game = 2
            self.save.save_to_file(2,"ph2_Air_game2")
        CAP = await self.read.Read_CAP(client)
        if CAP == 1:
            self.game = 3
            self.save.save_to_file(2,"ph2_CAP_game3")
            

    async def start_game(self, client):

        self.save.save_to_file(2,"ph2_start_game")

        if self.game == 4:
            return
        elif self.game == 1:
            result = await self.dance.play(client)
        elif self.game == 2:
            result = await self.guess.play(client)
        elif self.game == 3:
            result = await self.simon.play(client)

        if result == 1:
            call_action("ph2_congratulations")
            self.save.save_to_file(2,"ph2_congratulations")
        elif result == 0:
            call_action("ph2_nexttime")
            self.save.save_to_file(2,"ph2_nexttime")

        self.finish = 1
        print("end")
        return

        
    async def GameMenu(self, client):

        self.save.save_to_file(2,"ph2_intro")
        call_action("ph2_intro")
        

        while self.game != 4:
            await self.read.Read_char(client, 2)

            InputThread = threading.Thread(target=self.user_input)
            InputThread.start()

            print("game:", self.game)
            print("start:", self.start)
            print("finish", self.finish)

            if self.game == 0 and self.start == 0:
            # shake to start game
                self.start = await self.read.Read_IMU(client)
                self.save.save_to_file(2,"start_because_IMU")
            
            # only play choice if started but not yet game chosen.
            if self.start == 1 and self.game == 0:
                call_action("ph2_choose")
                self.save.save_to_file(2,"ph2_choose")
            
            if self.game == 0:
                await self.sensor_input(client)

            # if shake, DanceParty, squeeze GuessAnimal, hold SimonSays
           # so wait for interaction OR user input
            
            if self.game != 0 and self.finish == 0:
                await self.start_game(client)

            if self.finish == 1:
                IMU = await self.read.Read_IMU(client)
                if IMU > 0:
                    self.save.save_to_file(2,"IMU_activity_to_finish")
                    self.game = 4
               
        return

        
    