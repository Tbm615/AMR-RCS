RCS — Robot Control System
==========================

This dist/ directory contains two self-contained builds:

  rcs/                      -- Linux x86-64 build (run ./rcs/rcs)
  rcs-windows-x86_64/       -- Windows x86-64 build (run rcs.exe)
  rcs-linux-x86_64.tar.gz   -- Linux build packaged as a .tar.gz
  rcs-windows-x86_64.zip    -- Windows build packaged as a .zip

Neither build requires Python or ROS 2 on the operator machine.

Linux
-----
  cd rcs
  ./rcs

Config is written to ~/.config/RCS/.

Windows
-------
Extract rcs-windows-x86_64.zip to any folder, double-click rcs.exe, or run
from PowerShell:

  .\rcs-windows-x86_64\rcs.exe

Config is written to %APPDATA%\RCS\.

The .exe is a native PE32+ x86-64 binary (produced with Python 3.11 +
PyInstaller inside a wine 11 cross-build environment). It was verified
to load and run on this machine via wine; it will run natively on any
Windows 10/11 x86-64 machine.

Robot side
----------
On the robot's PC install and run rosbridge once:

  sudo apt install ros-humble-rosbridge-suite
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

A systemd unit template is in RCS/scripts/robot_setup_rosbridge.md.
