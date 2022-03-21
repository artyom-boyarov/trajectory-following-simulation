# Simulation of various trajectory following algorithms
This repository contains code I used in a research project to compare multiple lateral control algorithms.
## This repo
```results/``` Contains excel spreadsheets of results.<br>
```tracks/``` Contains waypoint files to be used.<br>
```main.py``` This file runs the simulation once for one lateral control algorithm.<br>
```simulation.py``` Contains the simulation class used with functions for steering.<br>
```master.py``` Contains the code which runs all control algorithms and collects results in csv format.<br>
All files can be run with ```--help``` to print out options
## Running
Note: Only python 3.6 has been shown to work with this code.<br>
To run install requirements first:<br>
```pip install -r requirements.txt```<br>
Then run each file, e.g.<br>
```python3.6 master.py```
