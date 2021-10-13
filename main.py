from tkinter import *
from tkinter import ttk

from simulation import Simulation
from multiprocessing import *


def main(algorithms):
    root = Tk()
    root.title("Choose algorithm to simulate")
    frame = ttk.Frame(root, padding=10)
    frame.grid()
    ttk.Label(frame, text="Welcome to the local path planning simulation.\nClick on an algorithm to start its simulation").grid(column=0, row=0)

    counter = 2
    for algo in algorithms:
        ttk.Button(frame, text=algo.name, command=Process(target=algo.start).start).grid(column=0, row=counter)
        counter += 1
    ttk.Label(frame, text="Created by Artyom Boyarov").grid(column=0, row=counter)
    root.mainloop()

if __name__ == '__main__':
    main([Simulation("Frenet frame"),
          Simulation("Dynamic window"),
          Simulation("Potential field")])
