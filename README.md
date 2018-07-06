# SnakeRobot
**Note: The code in this repository is part of an ongoing research project; portions of it are mostly ready for production use, but other portions are not, and documentation is not yet complete. In the future, stable versions will be made available as tagged releases.**

This repository contains the Arduino sketches for the snake robot constructed as a research project within Penn State Behrend's mathematics program ("Penn Snake Behrend"). These sketches address servos over PWM from the Arduino and accept digital user input that is assumed to originate from a 4-button keyfob; the sketches do not currently contain functionality for external sensors (though this may change in the future).

See [the project page on Dr. Joseph Paullet's faculty website](http://math.bd.psu.edu/faculty/paullet/robotics.html) for more general information about this project.

## Getting Started

### Installation

If you are using the Arduino IDE, then, as of version 1.8.5, the folder"SnakeRobotLibrary" will need to be copied into the IDE's "libraries" folder. On Windows, this can be done by running the batch file located at "Libraries\locallibinstall.bat"; this can also be done manually as follows:

1. Open the Arduino IDE and go to "File » Preferences".

2. Copy the "Sketchbook location" path and open it in another file explorer window.
3. Navigate to the "libraries" folder and copy the "SnakeRobotLibrary" folder (within "Libraries") there.

For other IDEs, adding the "SnakeRobotLibrary" folder (within "Libraries") to the list of include directories should allow the sketches to compile.

### Provided Sketches

The folders "Serpentine", "Rectilinear", and "Concertina" contain sketches for the respective gaits; see "README.md" within each of these folders for specific information on customizing the provided sketches for your robot.

The "Testing" folder contains testing sketches (currently only consisting of a sketch for testing servos' positioning at different angles).