# GRT 2019 Preseason
This repository will be used during the fall before the 2019 season for testing. This repository contains many improvements over GRTJava, including a special control loop for swerve and fieldmapping, sequential autonomous, and the removal of unused code.

## Notes
Please follow the instructions [here](https://wpilib.screenstepslive.com/s/currentCS/m/79833/l/932382-installing-vs-code) exactly. Make sure to install the Java Extension Pack so our code can look nice and consistent.

## Packages
Packages are now found under `src/main/java/frc/`

### config
No different from GRTJava. Contains global config class which parses config files. Config files are now `.txt` files found under `resources/frc/config`.

### controlloops
Contains `SwerveControl` which is the main loop for swerve. `SwerveControl` runs PID controllers for swerve as well as field mapping in one loop synced with sensor feedback. This package also contains synchronous PID and PIF controller implementations.

### fieldmapping
Contains a hierarchy of `PositionTracker`'s which will be removed when a single implementation is found to be the best. Will eventually contain actual field mapping.

### mechs
Contains mechs and `MechCollection`, a class that contains all of the mechs for easy passing to control modes.

### robot
Contains the main `Robot` class, as well as `Teleop` and `Autonomous`. `JoystickProfile` is a new class that will transform joystick inputs to improve drive handling. Currently contains some simple utility functions often used with joystick inputs.

### swerve
Contains the `FullSwerve` class, which should now only be controlled through `SwerveControl`. Also contains `Wheel`, `SwerveData`, and `NavXGyro` classes as they are mainly used by swerve. The other swerve modes have been removed.

### util
Contains `GRTUtil`, named such because there are so many other classes just called `Util`. `GRTUtil` contains utility functions which are not necessarilly used for joystick inputs.

## Other Files

### .vscode
Contains workspace settings and lauch configurations. The latter is useless as far as I know, the former will help make sure we all use the same settings.

### .wpilib
Contains a preferences file which includes our team number and programming language.

### gradle
Contains gradle wrapper `jar` file and preferences.

### .gitignore
Tells Git which files to never stage or commit. This one was automatically generated and seems quite comprehensive.

### build.gradle
Gradle build script for building and deploying. We probably won't mess with it.

### eclipse-formatter.xml
Some formatter information to make sure our code is all formatted the same way.

### gradlew
Gradle wrapper script for MacOS/Linux. use this to call gradle commands.

### gradlew.bat
Gradle wrapper script for Windows. use this to call gradle commands.

### README.md
This document you are currently reading.

### settings.gradle
Some more Gradle settings. I guess you can never have too many.