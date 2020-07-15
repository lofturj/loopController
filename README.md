# TF3LJ / VE2LJX Magnetic Loop Antenna Controller

The loop controller automatically tunes a magnetic loop antenna, using a variable capacitor, maintains tuning as the VFO changes, and much, much more.

A complete description as well as documentation is available at Loftur's [main loop controller page](https://sites.google.com/site/lofturj/to-automatically-tune-a-magnetic-loop-antenna)

Discussion and community support can be found at [groups.io](https://groups.io/g/loopController)

## Getting Started

These instructions will get you a working environment to compile and upload the controller firmware to a Teensy 3.2 microcontroller.

### Prerequisites

**For Visual Studio Code and Platformio**
- Install [VS Code](https://code.visualstudio.com/download).
- Install the Platformio extension from [the VS code marketplace](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).

**For Arduino IDE**
- Install the [Arduino IDE](https://www.arduino.cc/en/main/software)
- Install the [Teensyduino extension](https://www.pjrc.com/teensy/teensyduino.html)
  - Select all libraries when installing.
- Install the [TMCStepper](https://github.com/teemuatlut/TMCStepper.git) library.
  - Download the repository as a .zip file. Name it TMCStepper.zip.
  - In the Arduino IDE, navigate to Sketch -> Include Library -> Add ZIP library and select the ZIP file downloaded.

### Installing

Connect the teensy to the computer running your development environment.

**VS Code**
Open the repository folder in VS Code. Platformio should load and install all required modules automatically.

From the platformio menu, click Build and/or Upload.

**Arduino IDE**
Open ML_v4.ino in the Arduino IDE

Click the Verfy (check box) or Upload (arrow) buttons in the arduino IDE.
- Verify compiles the code.
- Upload compiles and uploads the code to the teensy.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Loftur E. Jónasson - TF3LJ / VE2LJX** - *Creator of this project, both hardware and code* - [Loftur's Radio Projects](https://sites.google.com/site/lofturj/)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the GNU General Public License License, Version 3 - see the [gpl.txt](gpl.txt) file for details

## Acknowledgments

