# How to use dockerfiles to build SLAM project

## Build Base environment

Base environment is a docker image that contains pre-built 3rd party libraries which are required to run SLAM.

To build Base environment, use the command below:

`sudo docker build --no-cache --progress=tty --force-rm -f base.dockerfile -t slam:base .`

## Build SLAM modules

To build SLAM modules, use the command below:

`sudo docker build --no-cache --progress=tty --force-rm -f modules.dockerfile -t slam:modules .`

---

# How to use X11 forwarding for docker development

Before using docker for development, you need X11 display forwarding software.

- If you use Windows, you will need XAuthority.
- If you use MacOS, you will need XQuartz
- If you use Linux, you already have X11 display forwarding software in system.

If you use CLion Docker integration kit, you don't need to run X11 forwarding.
