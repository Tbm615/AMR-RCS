Download the file for your computer. You don't need Python or ROS 2 on the operator PC.

| File | For |
|---|---|
| `rcs-windows-x86_64.zip` | Windows 10/11. Unzip, run `rcs\rcs.exe`. |
| `rcs-windows-x86_64-console.zip` | Same app; also opens a console window with logs, for troubleshooting. |
| `rcs-linux-x86_64.tar.gz` | Linux x86-64. `tar xzf rcs-linux-x86_64.tar.gz && ./rcs/rcs` |

The robot must run `rosbridge_server` on port 9090. Setup and everything else are in the **[User guide](https://github.com/b4lanc3/AMR-RCS/blob/main/docs/USER_GUIDE.md)**.

### Changes in v0.2.1

- The local costmap was drawn in the wrong place. Nav2 keeps it in the `odom` frame, and it is now placed on the map through TF.
- Costmap colors are easier to read: obstacles are solid, and inflation fades out instead of covering the whole corridor.
- Redesigned look: no more dark patches behind labels, proper arrows on drop-downs and number fields, clearer Status and Teleop pages.
- **Center on robot** and **Whole map** buttons on every map. Large maps open fitted and keep your zoom when the map updates.
- Mission waypoints are labeled on the map (`1 Pick-up`, `2 Drop-off`…). The Missions map shows Nav2's planned path.
- The robot's yaw is shown in [-π, π].
- Screenshots in the guide are taken from the app driving real Nav2.

### Changes in v0.2.0 (v0.1.0 crashed once the robot moved, don't use it)

- Fixed the crash: PySide6 6.12 corrupts memory with the map widgets, so the build now uses PySide6 6.8.3.
- Teleop no longer floods `/cmd_vel_safe` with zero commands when idle. That used to block Nav2.
- E-STOP also cancels the Nav2 goal and the running mission, and publishes `/amr/estop`.
- The lidar position is read from `/tf_static`, so the scan lines up with the map.
- The Navigation page shows the real Nav2 state, distance and time remaining, and warns when Nav2 doesn't respond.
- Missions wait for Nav2 to finish each goal and retry dropped or aborted goals.

### How it was tested

- On real Windows and Linux machines (GitHub Actions), the built app connects, subscribes to every topic, and stays silent while idle.
- On a PC running this workspace's Nav2 launch on `map23_04` with a simulated base, through a real `rosbridge_server`, the app set the initial pose, sent goals from the map, canceled them, triggered E-STOP while driving, and ran a looping mission. All 28 checks passed.
- **Not yet tested on the real robot.**
