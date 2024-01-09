import time
from movements import controller

def call_action(input, audiotime=1000, actiontime=1000):
        action = controller(input)
        action.loop(audiotime, actiontime)
        time.sleep(2)