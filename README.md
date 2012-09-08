# Servo Board

This is the code for an Arduino based servo controller board. The borad supports up to 16 servos.


## Requirements

- Arduino IDE
- SCons
- Python
- pyserial
- avrdude


## Setup

The project uses a build script to enable development outside the Arduino IDE.

Open the "buildproject" file, edit all constant values to fit your system and Arduino board and
save the file.


## Build

To build the project, open a terminal in this directory and type:

	$ ./buildproject

To upload the binary to the Arduino type:

	$ ./buildproject upload


## Hardware
The hardware schematics are being developed on Upverter [here](http://upverter.com/tirithen/fa5b6d5bb2f373fc/servoboard/).


## License

The code is released under the GPLv3 license.
