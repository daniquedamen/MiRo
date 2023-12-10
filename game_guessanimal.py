# Guess the sound
# donkey - 1
# cow - 2
# dog - 3
# cat - 4
# lion - 5
from stream_audio import streamer
import random
import threading

from read_sensors import Read_Sensors

class GuessAnimal:

    def __init__(self):
        self.list = [0]
        self.animal = 0

        self.current_animal = 0
        self.prev_animal =[0]
        self.x = 0

        self.finish = 0

        self.read = Read_Sensors()

    def whichone(self):

        while self.list.count(self.animal) == True:
            self.animal = random.randint(1,5)
            print(self.animal)

        self.list.append(self.animal)

    def orderofsound(self):
        while self.x < 7:
            while self.prev_animal.count(self.current_animal) == True:
                self.current_animal = random.randint(1,5)

            self.prev_animal.append(self.current_animal)

            if self.current_animal == 1:
                intro = "donkey"
            elif self.current_animal == 2:
                intro = "cow"
            elif self.current_animal == 3:
                intro = "dog"
            elif self.current_animal == 4:
                intro = "cat"
            elif self.current_animal == 5:
                intro = "leeuw"


            audio2 = streamer(intro)
            audio2.loop()
            self.x = self.x+1


    def play(self):
        while self.finish == 0:
            # which one are we playing
            self.whichone()

            # start instruction audio
            if self.animal == 1:
                intro = "intro_donkey"
            elif self.animal == 2:
                intro = "intro_cow"
            elif self.animal == 3:
                intro = "intro_dog"
            elif self.animal == 4:
                intro = "intro_cat"
            elif self.animal == 5:
                intro = "intro_lion"

            audio = streamer(intro)    
            audio.loop()

            PlayMusic = threading.Thread(target=self.orderofsound)
            PlayMusic.start()

            if self.animal == self.current_animal:

                # shake if you hear the correct animal.
                # if child continously shakes, nothing happens. TO DO
                if self.read.Read_IMU() == 1:
            
                    print("goed") # sound
                    audio3 = streamer("goodjob")
                else:
                    print("sorry try again")
                    audio3 = streamer("tryagain")

                audio3.loop()
                self.finish = 1

        print("animal loop finished")


if __name__ == "__main__":
    classGuess = GuessAnimal()
    classGuess.play()
