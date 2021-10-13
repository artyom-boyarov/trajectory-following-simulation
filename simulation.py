
class Simulation:
    def __init__(self, name):
        self.name = name

    def start(self):
        self.on_start()
        while True:
            self.on_loop(10)

    def on_start(self):
        print("WARNING: Attempting to run an empty simulation")


    def on_loop(self, delta_time):
        print("WARNING: Attempting to run an empty simulation")