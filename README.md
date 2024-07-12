# Simulation of various trajectory following algorithms
This repository contains code for a simulation that shows a bus traveling along a trajectory in an urban environment. The bus uses a specified control algorithm to follow the trajectory. Implemented control algorithms include Stanley control, Pure Pursuit control, and some variations.
<br>
I used this simulation in my first research project where I compared the effectiveness of multiple lateral control algorithms in driving an autonomous bus around a track in an urban environment. The article for this research is available at <a>https://emerginginvestigators.org/articles/22-104</a>. See <a href="Abstract.md">Abstract.md</a>.
<br>
Feel free to use this code for your own research or experimenting, or to see implementations of the Stanley and Pure Pursuit control algorithms.

## This repo
```results/``` Contains excel spreadsheets of results.<br>
```tracks/``` Contains waypoint files to be used.<br>
```course_images/``` Contains diagrams of the courses used in the simulation. <br>
```simulation.py``` Contains the simulation class used with functions for steering.<br>
```main.py``` Contains the code which runs all control algorithms, or just one, and collects results in csv format.<br>
```lateral_controllers.py``` Contains implementations of the control algorithms used.<br>
```all.sh``` A script to run the simulation across all courses and all algorithms and all speeds.<br>

## Running
Create virtual environment: <br>
```
python -m venv .venv
source .venv/bin/activate
```
To run install requirements first:
```
pip install -r requirements.txt
```
Then run each file, e.g.
```
python main.py
```
