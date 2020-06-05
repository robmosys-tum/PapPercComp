# Papyrus4Robotics

## Structure

PapyrusNodes contains the following nodes:

- KnowRobWrapper
- other nodes to be added


## Instructions (Development)

The project can be build using

```bash
colcon build --symlink-install --packages-up-to knowrobwrapper
```

Another option is using Papyrus, but make sure to source the workspace before working on the distro

```bash
source /opt/ros/<ros2 distro>/setup.bash
. install/local_setup.bash
```

Then open papyrus in the shell, right click on the project, and then build project.

