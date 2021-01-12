# SUTDoggo

SUTDoggo is a highly agile robot designed to be an accessible platform for legged robot research. We adopt the Stanford Doggo for this project. This could serve to inspire fellow SOAR members in robotics.

Credits to the [original Stanford Doggo repository](https://github.com/Nate711/StanfordDoggoProject)!

This repository will be divided into three parts to house the different parts of the SUTDoggo:

1. This main root folder will contain the main project files such as the BOM, updated CAD files, circuit diagram for electronics, general guidelines, project goals and plans, FAQ, etc.
2. The ODrive subfolder will contain the modified version of the ODrive firmware (merged v0.4.11 of the official factory firmware with Nate's code customizations).
3. The Doggo subfolder will contain the custom Doggo firmware. Currently, it is still using the default Doggo codebase. Future plan would be to refactor this codebase into a ROS workspace to enable ROS package integrations.

## Table of Contents <a name="table-of-contents"></a>

- [SUTDoggo](#sutdoggo)
  - [Table of Contents](#table-of-contents)
  - [Stakeholders & Maintainers](#stakeholders-and-maintainers)
  - [Objectives & Deliverables](#objectives-and-deliverables)
    - [Goal Deadlines & Status](#deadlines)
    - [Extra Objectives](#extra-objectives)
  - [Features & Technologies](#features-and-technologies)
    - [Features](#features)
    - [Technologies](#technologies)
  - [Prerequisites](#prerequisites)
  - [Demo](#demo)
  - [Troubleshooting & FAQ](#troubleshooting-and-faq)
  - [Future Plans & Implementations](#future-plans-and-implementations)
  - [Components](#components)
    - [Mechanical](#mechanical)
    - [Controller/Driver](#controller-driver)
    - [Sensors](#sensors)
  - [Additional Resources](#additional-resources)

## Stakeholders & Maintainers <a name="stakeholders-and-maintainers"></a>

- Project Lead: [James Raphael Tiovalen](https://github.com/jamestiotio) (Maintainer)
- Mechanical Engineers:
    - [Kevin Ma Yuchen](https://github.com/Kevinskwk)
    - Zhi Cong
    - Yoong Hao
    - [Dody Senputra](https://github.com/ulaladungdung)
- Electrical Engineers:
    - [Chung Wah Kit](https://github.com/sdencanted)
    - Gerald Wong
- Software Developers:
    - [James Raphael Tiovalen](https://github.com/jamestiotio)
    - [Dody Senputra](https://github.com/ulaladungdung)
    - [Kevin Ma Yuchen](https://github.com/Kevinskwk)
    - [Chung Wah Kit](https://github.com/sdencanted)
- Mentors:
    - [@methylDragon](https://github.com/methylDragon)
    - [@Fasermaler](https://github.com/Fasermaler)
    - Bryan Kong
    - Shi En
- Supervisor: Prof. Tan U-Xuan

## Objectives & Deliverables <a name="objectives-and-deliverables"></a>

### Goal Deadlines & Status <a name="deadlines"></a>

- Mechanical Assembly (Status: In Progress ⌛)
- Electrical Assembly (Status: In Progress ⌛)
- Software Adjustment (Status: Completed ✔️)
- Teleoperation Calibration (Status: Not Yet ❌)

### Extra Objectives <a name="extra-objectives"></a>

- Implement ZED Stereo Camera
- Implement autonomous navigation like LinoRobot
- Complete documentation of Doggo's navigation stack

## Features & Technologies <a name="features-and-technologies"></a>

### Features <a name="features"></a>

The Doggo boasts locomotion capabilities, including:

- Trotting
- Jumping
- Backflipping
- (More acrobatic moves/maneuvers to come?)

### Technologies <a name="technologies"></a>

The Doggo's firmware is written in C++. The Doggo is equipped with RF-based communication protocol using the XBees. In implementing autonomous navigation, we would replace the XBees with a Raspberry Pi equipped with the ROS navigation stack.

## Prerequisites <a name="prerequisites"></a>

Before diving into this project, make sure that you are at least familiar with:
- Arduino & C++
- Mechanical Manufacturing & Assembly Techniques
- Electrical Circuits (Microcontrollers, Motors & Encoders)

## Demo <a name="demo"></a>

> Include some videos, GIFs and/or pictures in this space! (And some brief walkthrough.) [color=#e053c1]

## Troubleshooting & FAQ <a name="troubleshooting-and-faq"></a>

*# Tick when the problem is solved.*
> After solving a problem, rephrase the problem to include the solution so as to suit this troubleshooting section. [name=James R T] [color=#907bf7]

#### Unsolved

- Current Doggo is flimsy and "limps over" when walking. Status is not ready for deployment. Need to attach the silicone leg molds since we suspect that there is almost no friction preventing the Doggo from slipping. If the molds are not compatible, need to recreate the molds. After molds are attached, recalibration is needed and some firmware constants might need to be changed. If done properly, the Doggo should be able to trot and walk properly. Only by then, testing for backflipping can begin.
- Once Doggo is working properly, update this README and populate with relevant details, especially these sections: [Objectives](#objectives-and-deliverables), [Demo](#demo), [Troubleshooting & FAQ](troubleshooting-and-faq) and [Future Plans & Implementations](#future-plans-and-implementations).

#### Solved

- The M3 10mm and M2.5 8mm screws are not long enough to fix the encoder mount and T-Motor onto the side panel.
    *Solved by buying new screws.*
- The holes on the 3D printed parts on the electronic plate support are too big to fit the insert. Need to reprint the part or buy bigger inserts.
    *Solved by buying new inserts.*
- The spacers between gears and motors need to be laser cut.
    *Solved by laser cutting the parts.*
- The leg parts fabricated according to the CAD files don't include the holes needed to tighten the leg and the axial
    *Solved by trilling holes and using set screws to tighten up.*
- The belt is not tight enough and it could slip in cases of high angular velocity of the motors.
    *Solved by attaching the brackets on top of the belt assembly (original design by Stanford). However, this is a short-term solution since it might loosen over time or break.*

#### FAQ

##### The sub-folders are empty!
Run this to populate the folders.
```shell
git submodule update --init --recursive --remote
```
##### How do I configure the ODrive parameters for Doggo?
We use the script ```https://github.com/Nate711/ODrive/blob/master/tools/doggo_setup.py```
##### How are the legs numbered?
Leg0 is the front left, leg1 is the back left, leg2 is the back right, and leg3 is the front right.
##### How do I calibrate the legs?
This is a sore spot for the robot right now. When you power on the robot, all the driven linkages (ie top links) need to be as horizontal as possible. If everything is set up correctly, the motors will then begin their calibration routine which involves them rotating about 120 degrees to one side and then back. After this, the robot is ready.
##### Can it turn?
Yes and no. It turns very slowly with the 'Y' command. The turning speed is set by 's [desired step difference]' where the step difference should be around -0.1 to 0.1 at the most.
##### Which way should the IMU point?
There are little markings on the BNO080 board which indicate which way x is positive. Align that direction with the forward direction of the robot. The forward direction is towards the motors connected to ODrives 0 and 3. 
##### How are the ODrives connected to the Teensy?
We use serial connections between the Teensy and ODrives so for each ODrive you'll need to connect Teensy RX to ODrive TX and Teensy TX to ODrive RX. Since there are four ODrives, you'll have to connect up four sets of these serial connections!

In code we have these definitions which may help with wiring:
```cpp
HardwareSerial& odrv0Serial = Serial1;
HardwareSerial& odrv1Serial = Serial2;
HardwareSerial& odrv2Serial = Serial3;
HardwareSerial& odrv3Serial = Serial4;
```

Here's also a little chart made by HelloPlanet!
```
Teensy3.5 --- Logical Function --- Component
P0 - RX1 - ODrive1 (odrv0)
P1 - TX1 - ODrive1 (odrv0)

P7 - RX3 - ODrive3 (odrv2)
P8 - TX3 - ODrive3 (odrv2)

P9 - RX2 - ODrive2 (odrv1)
P10 - TX2 - ODrive2 (odrv1)

P31 - RX4 - ODrive4 (odrv3)
P32 - TX4 - ODrive4 (odrv3)
```
##### How is the Xbee wired?
We connect it via serial to Serial5. We also have a definition in ```config.h``` called ```USE_XBEE``` that determines whether the code should use the actual USB port on the Teensy or the Xbee for communications. 
##### How is the IMU wired?
We connect it via SPI with the following defines.
```cpp
//Pins for BNO080 IMU
#define SPI_CS_PIN 15
#define SPI_WAK_PIN 14
#define SPI_INTPIN 17
#define SPI_RSTPIN 16
```
##### How does the relay get wired up?
We now use a solid-state relay https://www.digikey.com/product-detail/en/sensata-crydom/D06D100/CC1520-ND/353618 with the output terminals in series with the main power and the input terminals connected to an external battery with a switch. We have the switch wired up in serial so that pressing the switch will turn on/off the relay and thus the robot's main power.
##### The CAD doesn't download correctly
This is unfortunately not a problem on our end but on Fusion's. Fusion customer service has in the past been able to fix corrupted uploads on their server.

#### Risk Assessment

- Electronics should be handled with care.

- Fabrication will involve physical and ergonomics hazard, so handle mechanical work with care. 

- The Doggo should be placed in a controlled environment during testing. This is because it could hurt others when it jumps or backflips. 

## Future Plans & Implementations <a name="future-plans-and-implementations"></a>

We plan to implement autonomous navigation using the ROS navigation stack with a ZED camera. We also plan to add more acrobatic moves to the Doggo. Another possible implementation is to add OpenPose feature into Doggo.

### Improvement Points for Doggo Mk. II

> Minimize rabz gotchas and hacky stuffs.

- Fix the Minitactor implementation.
- Enable full IMU implementation (not just for recording backflips).
- Add space to allow headers to be soldered to the encoders (solving the pressure issue).
- We would need to redesign the whole belt assembly in order to be able to variably tighten the belt. This would lead to a more sustainable tightening mechanism, instead of using a 3D-printed part that could break or bend over time (prevent the CF from bending).
- Change the gear-motor adapter material from acrylic to metal (by CNC).
- Implement a better cable management system (shorten cables, tidying up cable space, use JST connectors, etc).
- Implement a proper mounting of the electronics to the electronic CF plate (instead of only by using cable ties and tapes).
- Increase encoder reliability by securing them in their corresponding positions with better methods.
- Change all ODrives from 24V to 48V to prevent any burning (since the battery's voltage can rise to about 25V).
- Convert the Doggo firmware into a ROS workspace.
- Create a customized electronic circuit optimized for Doggo (perfboard or PCB) [OPTIONAL].
- Add a charging circuit [OPTIONAL].

## Components <a name="components"></a>

SOAR operates in Singapore, which has a significantly different economic landscape compared to Stanford in USA. Thus, some of the original parts were a bit difficult to find and we opted for alternatives. A full list of the components that we used will be included in the detailed BOM.

[BOM](https://sutdapac-my.sharepoint.com/:x:/g/personal/soar_club_sutd_edu_sg/EaF5epe0R2xKlXmtg6U--s0B9OhUuDgEI9kDiBbzD2ODvg?e=lZnee1)

> We will do up a new, improved, more detailed and more reliable BOM. Also, any ideas on how to improve the organization of these components? [name=James R T][color=#ea9b12]

### Mechanical <a name="mechanical"></a>

#### Gear Brace

The triangular gear brace might seems a bit big and doesn't fit the gears well. But it is actually for pushing the gears away a bit so that the belts can be tightened.

#### Legs & Coaxial

The leg parts didn't come with holes to fix them to the coaxial parts. So we need to drill holes ourselves and use set screws or dowel pins to fix the legs.

#### Spacers between Gears and Motors

There should be spacers between the gears and motors in order to mount the gears onto the motors.

### Controller/Driver <a name="controller-driver"></a>

#### Teensy

The code does not have a .ino file. Instead we use Platformio to compile all the code in the src and lib folders and upload it to the teensy.

- Install VSCode and PlatformIO

    - Download and install official Microsoft Visual Studio Code. PlatformIO IDE is built on top of it
    - Open VSCode Package Manager
    - Search for ```official platformio ide extension```
    - Install PlatformIO IDE.

- Get the PlatformIO sketch
    - Clone https://github.com/Nate711/StanfordDoggoProject
    - Open VSCode 
    - Open the Doggo folder in VSCode
    - Open `config.h` and comment out line 28 (`#define USE_XBEE`)

        > This is done to test the motors through USB when the XBee is not configured/connected yet.
    - Take note that some serial monitors like XCTU may use Carriage Return for line breaks, which is not recognised by the Teensy code. To enable it, change line 21 of Doggo/src/usb_serial.cpp:`if (c == ';' || c == '\n' || c == '\r') {`
    - Plug in the teensy using a USB cable
    - Press Build (tick in the bottom left bar of vscode)
    - Press Upload (right arrow in the bottom left bar of vscode)
    - Open the Serial Monitor (electric plug icon in the bottom left bar)
> Alternatively, run `platformio run --target upload` on the root Doggo folder that contains the `platformio.ini` file.

> In Doggo's `src` folder, change the original `#include` library lines of `datalog.cpp` and `imu.cpp` from `"arduino.h"` to `"Arduino.h"` and of `debug.h` from `"ChRT.h"` to `"ChRt.h"`.
- Connect the UART cables from teensy to ODrives
    - connect TX to RX and vice versa
    - serial pins for teensy as described in https://www.pjrc.com/teensy/td_uart.html

    - Teensy 3.5 ports listed:

        Serial port | RX | TX
        --- | --- | ---
        1 | 0 | 1
        2 | 9 | 10
        3 | 7 | 8
        4 | 31 | 32
        5 (for XBee controller) | 34 | 33
    - Odrive UART TX / RX
        TX | RX
        --- | ---
        GPIO1 | GPIO2
- Open a serial monitor( Arduino IDE or PlatformIO serial monitor should be fine )
- Available serial commands:
    -    Use a serial monitor (we use the Arduino one) to send over these commands to Doggo in order to set the behavior or to change parameters.
    - Changing behavior
    - General behaviors:
        - 'S': Put the robot in the STOP state. The legs will move to the neutral position. This is like an software e-stop.  
        - 'D': Toggle on and off the printing of (D)ebugging values.
        - 'R': (R)eset. Move the legs slowly back into the neutral position.

    - Working gaits:  
        - 'B': (B)ound. The gait is currently unstable.  
        - 'E': Danc(e). Make the robot do a little bouncy dance.    
        - 'F': (F)lip. Execute a backflip.  
        - 'H': (Hop). Causes the robot to start making small vertical hops.  
        - 'J': (J)ump. A full-torque upwards leap.  
        - 'T': (T)rot. Begin a forward trot. This is currently the only working forward gait.  

    - Available, but not working:
        - 'W': (W)alk. Does not work currently.  
        - 'P': ( P )ronk. Much like hop, but this one doesn't work.  

    - Changing gait properties:
        - 'f {float}': Set the gait frequency in Hz.  
        - 'l {float}': Set the stride length in meters.  
        - 'h {float}': Set the stance height (neutral height) in meters.  
        - 'u {float}': Set the distance (in meters) that the leg travels above the neutral line during a stride.  
        - 'd {float}': Set the distance (in meters) the leg travels below the neutral line during a stride.  
        - 'p {float}': Set the proportion of time which the leg is below the neutral line versus above it.    
    - Changing compliance (gains):
        - 'g {float} {float} {float} {float}': Set the compliance gains. The ordering is {kp_theta} {kd_theta} {kp_gamma} {kd_gamma}. A good default is 'g 80 0.5 50 0.5'.  

### Sensors <a name="sensors"></a>

#### Contactor
![alt text](https://i.imgur.com/AFyg6nn.png "How to connect the Contactor (MZJ 100A)")


#### Encoder

- Make sure the magnet and encoder have the same center as the T-motor axle 
- Commonly seen in e-skateboards

#### Coaxial

- The inner and outer leg shafts are supposed to be tight fitting with the bearings. Due to fabrication inaccuracy, the shafts might be a bit thicker than the bearings. When putting them together, must be careful not to break the bearings.

#### ODrive

- Although the original Doggo uses ODrive 3.5 24v, we ended up using different models with success:
    - ODrive 3.5 48v
    - ODrive 3.6 24v

- ##### Testing of the ODrive

    -  Follow this: https://docs.odriverobotics.com/

    - Run these and hope the odrive usb is detected
        ```
        echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/50-odrive.rules
        sudo udevadm control --reload-rules
        sudo udevadm trigger # until you reboot you may need to do this everytime you reset the ODrive
        ```

    - If the ODrive is not connecting, try changing the power source. A dim light on the ODrive indicates insufficient power.

    - Always calibrate when the ODrive is first started
        ```
        odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE 
        ```

    - Reccomended P gain
        ```
        odrv0.axis0.controller.config.pos_gain = 0.05
        ```

    - Manual control is only enabled when closed loop control is enabled:
        ```
        odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        ```


    - try moving the motor
        ```
        odrv0.axis0.controller.pos_setpoint = 1000
        ```

- #### Setup (in Linux)

    - #### Installing custom firmware

        - Clone Nate's Odrive repository https://github.com/Nate711/ODrive
        - Navigate to ODrive/Firmware
        - Copy `tup.config.default` as 'tup.config'
        `cp tup.config.default tup.config`
        - Edit the value of `CONFIG_BOARD_VERSION` according to the ODrive you are using:
             ODrive model | Value | Remarks
            --- | --- | ---
            3.1 | v3.1 | Untested
            3.2 | v3.2 | Untested
            3.3 | v3.3 | Untested
            3.4 24v | v3.4-24V | Untested
            3.4 48v | v3.4-48V | Untested
            3.5 24v | v3.5-24V | 
            3.5 48v | v3.5-48V | 
            3.6 24v | v3.6-24V | Requires change to Tupfile.lua
            3.6 24v | v3.6-56V | Requires change to Tupfile.lua / Untested
            -    Note: for ODrive 3.6 and later: An addition to Tupfile.lua is required (based on the main branch of the ODrive repo).
            ```
            elseif boardversion == "v3.5-48V" then
                boarddir = 'Board/v3'
                FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
                FLAGS += "-DHW_VERSION_VOLTAGE=48"
            ### ADDITIONAL PART STARTS HERE
            elseif boardversion == "v3.6-24V" then
                boarddir = 'Board/v3'
                FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=6"
                FLAGS += "-DHW_VERSION_VOLTAGE=24"
            elseif boardversion == "v3.6-56V" then
                boarddir = 'Board/v3'
                FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=6"
                FLAGS += "-DHW_VERSION_VOLTAGE=56"
            ### ADDITIONAL PART ENDS HERE
            elseif boardversion == "" then
            ```
        - Do `make`
            
            - This may require you to `sudo apt install tup`
        - The firmware can be found in ODrive/Firmware/Build/
        - For users of ODrive 3.4 and lower please refer to https://docs.odriverobotics.com/odrivetool.html#device-firmware-update for firmware flashing instructions
        - For ODrive 3.5 and higher, run the following:
        ```
        odrivetool dfu path/to/ODrive/Firmware/build/ODriveFirmware.hex
    ```
    
- #### Setting parameters
    
        - Nate's default configuration script can be found here:
        ```
        python ODrive/tools/doggo_setup.py
        ```
        - By default the ODrive takes no action at startup and goes to idle immediately. In order to change what startup procedures are used, set the startup procedures you want to True. The ODrive will sequence all enabled startup actions selected in the order shown below.
            ```
            odrv0.axis0.config.startup_motor_calibration = True
            odrv0.axis0.config.startup_encoder_offset_calibration = True
            odrv0.axis0.config.startup_closed_loop_control = True
            ```
        - Repeat for axis1.
            ```
            odrv0.axis1.config.startup_motor_calibration = True
            odrv0.axis1.config.startup_encoder_offset_calibration = True
            odrv0.axis1.config.startup_closed_loop_control = True
            ```
        - Save and reboot
            ```
            odrv0.save_configuration()
            odrv0.reboot()
            ```
    - Refer to the teensy section on how to connect the ODrive to the Teensy via UART
    
    - take note that ```odrv0.save_configuration()``` only works once per reboot!

#### XBee

1. Follow [this tutorial](https://learn.sparkfun.com/tutorials/exploring-xbees-and-xctu/all).
2. Install [X-CTU](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu#productsupport-utilities), [Digi USB RF Drivers](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu#productsupport-drivers) and [FTDI Drivers](https://www.ftdichip.com/Drivers/VCP.htm) for your respective operating system.
3. Connect both XBees to your master computer for initial configuration.

    > X-CTU can only detect XBees that do not have wires soldered to the LED pins yet.

4. Follow the tutorial to define your unique network (CH, ID, DH, DL and MY).
5. Done! You can proceed to connect one XBee to your master computer and the other XBee to the Teensy.
    a. Connect RX(Xbee) to RX(Teensy Serial port 5) and TX(Xbee) to TX(Teensy Serial port 5)
    b. when using XCTU console to send commands to the teensy, use `;` to simulate a line break to send commands

#### IMU

- The particular IMU used for our Doggo is the CJMCU-80
- information taken from page 10 of https://cdn.sparkfun.com/assets/1/3/4/5/9/BNO080_Datasheet_v1.3.pdf
- How to connect the IMU to the teensy:
    - Teensy 3.5 port | IMU port | meaning
        --- | --- | ---
        3.3V | 3.3V | Power
        GND | GND | Ground
        11 | SA0 | MOSI (data input)
        12 | SDA | MISO (data output)
        13 | SCL | SCK (SPI clock)
        14 | PS0 | Protocol Select 0 / WAKE (also needs to be turned high on boot to activate SPI)
        15 | PS1 | Protocol Select 1 (needs to be turned high on boot to activate SPI)
        16 | RST | Reset pin
        17 | INT | Interrupt pin ( to bug the teensy )
        
## Additional Resources <a name="additional-resources"></a>

- [ODrive Robotics](https://docs.odriverobotics.com/)
- [Sparkfun XBee](https://learn.sparkfun.com/tutorials/exploring-xbees-and-xctu/all)

# Software

## Prerequisites

### ODRIVE Development Tools

The recommended tools for ODrive development are:

- **make**: Used to invoke tup
- **Tup**: The build system used to invoke the compile commands
- **Python**: For running the Python tools
- **ARM GNU Compiler**: For cross-compiling code
- **ARM GDB**: For debugging the code and stepping through on the device
- **OpenOCD**: For flashing the ODrive with the STLink/v2 programmer

Linux (Ubuntu 18.04)

``` 
$ sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
$ sudo apt-get update
$ sudo apt-get install gcc-arm-embedded
$ sudo apt-get install openocd

// To install Tup
$ sudo add-apt-repository ppa:jonathonf/tup 
$ sudo apt update 
$ sudo apt install tup
```

*source:* [link](https://docs.odriverobotics.com/developer-guide.html)

#### ODrive Default Firmware

```
// Clone Nate's ODrive Firmware
$ git clone https://github.com/Nate711/ODrive.git

$ cd ODrive/Firmware
$ make
// if all blue then good
// if got red, then check error message

$ cd build

// Flash firmware's hex file to odrive
$ sudo odrivetool dfu ODriveFirmware.hex
```
=======
**I think that about wraps everything up! Let me know if you have questions, and I&rsquo;ll also try to update the repo if people are interested in more details.**
