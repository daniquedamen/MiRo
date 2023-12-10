'''
07-11-23 - Danique Damen (d.damen-1@student.utwente.nl)
Connect via BLE to the server (MC) to receive data from its sensors.
'''
# libraries
from bleak import BleakClient, BleakScanner
import asyncio
import uuid

# own code
from interaction_ph1 import ReadInteraction
from games_ph2 import Games
from stories_ph3 import Stories

class PlushieReceiver:

    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.device = None

        # initialize read class
        self.Interact = ReadInteraction()
        self.Games = Games()
        self.Stories = Stories()
        

    # function to scan for BLE devices. When the IP Adress of the MC is found, return device
    async def find_plushie(self):
        scanner = BleakScanner()
        while self.device == None:
            dev = await scanner.discover()
            for i in range(len(dev)):
                if dev[i].name == "Plushie": # IP Address is "0F:9F:54:70:D1:14":    
                    self.device = dev[i]
                    print('Found Plushie IP')
                    return
            print("Plushie not found. Trying again.\nIf this is taking too long; try restarting Plushie.")

    
    # device was found, connect as client.
    async def connect_plushie(self):
        async with BleakClient(self.device.address, loop=self.loop, timeout=100) as client:
            print("Conne1cted to Plushie")

            services = await client.get_services()
            for service in services:
                if service.uuid == "4bd03949-19ce-466a-9abc-c53119e26f87":
                
                    await self.Interact.read_plushie(client)

                    await self.Games.GameMenu(client)
                    
                    await self.Stories.MagicWandStory(client)
                    
            
if __name__ == "__main__":
    receiver = PlushieReceiver()
    receiver.loop.run_until_complete(receiver.find_plushie())
    receiver.loop.run_until_complete(receiver.connect_plushie())
    print("Tests done")


    
    
    


    
    
    
    


