# hri learning
====
A package for analysing and learning HRI trajectories 

read_data.py
-------------

HRI dataset from Lincoln is located in ~/STRANDS/HRI_study_data/hri-study


The content is two directories 'crossing' and 'passby' where the robot
always drove from the same starting position to the same goal in both
cases. The human however once started opposite of the robot walking
towards it for the passby and offset by 90 degrees crossing its path for
the path crossing. Each directory contains the subdirectories p5 to p16
for the participants where each of these subdirectories contains several
files. The interesting ones are the .csv files. Naming convention:
`p<participant_number>_<trial>.csv` where participant number is the same
over all trials for the same participant and trial is a number from 0 to
9. Each participant repeated each interaction 10 times so there should
be 10 path crossing and 10 passby for each participant. Inside the files
are the positions of the human, the robot, and the robots goal in map
coordinates. Example for one timestamp:


-4.9450403744    -1.1905210114    goal    human    robot
-1.4684446938    -8.7528066635    -1.1772563458    -3.5247952912


The header is inherently unordered due to the python dict writer not
respecting any order. `agent?` is the name of the agent and `x?` and
`y?` are the corresponding coordinates. Hence the human should always be
`agent2` with the coordinates `x2` and `y2`. the robot is always
`agent1` and the goal is always `agent3`.

