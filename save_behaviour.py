import csv
import time

from datetime import datetime


class Save:

    def __init__(self):
        self.phasefile = None
        self.filename = None
        self.starttime = None

    def save_to_file(self, phase, input, var=""):

        if self.phasefile != phase:

            self.filename = ("/home/ddamen/mdk/mdk/bin/shared/HELPER/data/" + time.strftime("%Y%m%d_%H%M") + "beh_ph_" + str(phase) + ".csv")

            file = open(self.filename, 'a')

            writer = csv.writer(file)

            writer.writerow(["start at:" + time.strftime("%Y%m%d_%H%M")])
            writer.writerow([])
            writer.writerow(["min", "s", "elapsed_s", "input", "var"])
            file.close()

            
            self.starttime = time.time() # in nanoseconds
            self.phasefile = phase

        elapsedtime = (time.time() - self.starttime)
        elapsedtime = int(elapsedtime)

        now = datetime.now()
        s = now.second
        min = now.minute


        # write to file
        file = open(self.filename, 'a')
        writer = csv.writer(file)
        writer.writerow([min, s, elapsedtime, input, var])
        file.close()

        return