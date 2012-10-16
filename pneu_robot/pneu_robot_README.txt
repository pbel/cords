file:	pneu_robot_README.txt
author:	Paul Beliveau
date:	16 October 2012

Notes for usage of the M3 code customized for simulating the
Whitesides group's pneumatic quadruped robot.

Before trying to run this version of the code, familiarize yourself
with the M3 interactive simulator at:
http://www.uvm.edu/~ludobots/index.php/Main/Interactive

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The robot morhpology is controlled by constants in constants.h. There
are two ways to control the gait, from a controller file and
constants, or by embedded code in controller.cpp and constants. This
document describes control from an external file because that is the
most direct way to test a controller from the Whitesides group in the
simulator.

The gait is controlled by a combination of a text file whose path and
filename must be specified on the command line, and by constants in
constants.h. The number of time intervals ('NUM_TIME_INTERVALS') value
in constants.h *must* match in the number of lines in the control
file.

The controller files are developed by experimentation by the pneumatic
robot designers at Harvard and are saved in an Excel file. The Excel
file has some unused columns, and it must be converted to a text file
with a slightly different format to make it compatible with the M3
pneumatic robot code. 

The column functions for the quadruped are shown in the "walk" Excel
file reproduced below. (The first two lines are comments, not found in
the file.) 'X' indicates an unused column. 

//time	  LF   RF   X	 X    LR   RR	X    X	  body
//--------------------------------------------------------
750  	  0    1    0	 0    0	   0	0    0	  1
750	  0    0    0	 0    1	   0	0    0	  1
750	  1    0    0	 0    0	   0	0    0	  1
750	  0    0    0	 0    0	   1	0    0	  1

A '1' in a column other than the time column indicates that the
corresponding body part is pressurized during that interval. The time
column shows how long each state is maintained. (Note that the times
in milliseconds don't necessarily correspond to what is seen in videos
posted on the web.) When pressurized, the body part curves; when
unpressurized, it is flat.

The file above gets translated to the following for control of the
simulated robot, where the order of the robot segments in each line
is: body LF RF LR RR

1 0 1 0 0
1 0 0 1 0
1 1 0 0 0
1 0 0 0 1

The column for the body segment here is the leftmost column; the legs
are in the same order as in the Excel file.

Here, no times are specified. Each line represents one time interval;
the time interval value is set in constants.h,
'TIME_STEPS_PER_INTERVAL'. To create a control file from an Excel file
with unequal times, the TIME_STEPS_PER_INTERVAL value must be the GCD
of the specified time intervals in the Excel file and the lines in the
control file must be duplicated until the total time interval is the
same as in the Excel file. See the file "undulate.txt" for an
example. 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To run the pneumatic robot with a command file, use the command
line option '-fc', as follows

M3 -fc SavedFiles/walk.txt 

The simulator starts in design mode, waiting for the user to establish
a target for the robot. Evolution does not work in pneumatic robot
mode. However, in order to get to "file" mode, the user must first
establish a valid environment. Copy the existing robot and move the
copy out of the way, enter evolve mode (command: 'v'), then
immediately enter file mode with command '$'. The robot is red in
design mode, blue in evolve mode, and cyan in file mode.

Once the robot is in file mode, it will execute the commands in the
file specified by the 'fc' switch on the command line. The location of
the "target" robot and the light source have no effect on the
operation. 

It is essential that NUM_TIME_INTERVALS in constants.h match the
number of lines in the control file. Changing controllers, therefore,
always requires a re-compile.

There are two controllers implemented: walk and undulate. Both of
these came from controller information provided by Steve Morin at
Harvard. The pneumatic robot group at Harvard controls the physical
robots with a spreadsheet file. Examples of the format are shown in
the controller.cpp file in the methods "Motors_Set_Walk" and
"Motors_Set_Undulate". Note that the settings in these methods are
unused in file mode. 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The friction coefficient is set to a higher than typical value in
M3.cpp.

For the pneumatic robot, the motors are controlled in "motor groups".
Some of the code (e.g., the optimizer), however, refers to the groups
as motors because the original code didn't have the capability to
group motors.
