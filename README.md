# trep_omni
An example ros package using the Phantom Omni with trep.  The following are instructions on installing the Phantom Omni on Ubuntu.

***

## Phantom Omni Driver and OpenHaptics Toolkit Installation

These instructions were tested on Ubuntu 14.04 using a firewire card with the VT6315 controller.

### Required Packages:
* OpenHapticsAE\_Linux\_v3\_0
* Linux\_JUJU\_PPD

### Installation Steps:
Extract OpenHapticsAE\_Linux\_v3.0 to a local folder
Goto the OpenHaptics\_Linux\_v3\_0/PHANTOM Device Drivers/64-bit folder. Run:

    sudo dpkg -i phantomdevicedrivers_4.3-3_amd64.deb

Extract Linux\_JUJU\_PPD to a local folder
Copy  libPHANToMIO.so.4.3 in to /usr/lib.
Create symbolic links: 

    sudo ln -s  /usr/lib/libPHANToMIO.so.4.3  /usr/lib/libPHANToMIO.so 
    sudo ln -s  /usr/lib/libPHANToMIO.so.4.3  /usr/lib/libPHANToMIO.so.4

Copy the PhantomTest program from the JUJU PPD:

    sudo cp PHANToMConfiguration /usr/sbin/

Install the following dependencies.

    sudo apt-get install freeglut3-dev libmotif4 libglw1-mesa x11proto-dri2-dev libdrm2 libncurses5-dev

Create a driver symlink. In the /usr/lib/x86_64-linux-gnu directory:

    sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11 /usr/lib/x86_64-linux-gnu/libraw1394.so.8

Create a udev rule for the Phantom Omni

    sudo touch /lib/udev/rules.d/50-phantom-firewire.rules

Add the following line to 50-phantom-firewire.rules

    SUBSYSTEM=="firewire", ATTR{vendor}=="0x000b99", MODE="0660", GROUP="plugdev"

If the Phantom Omni is currently plugged in, restart driver with

    sudo modprobe -r firewire_ohci
    sudo modprobe firewire_ohci

At this point, test the Phantom Omni by running the configuration app

    PHANToMConfiguration

The Phantom Model field must be set to Omni, then click OK.  Next, run the Phantom Test app

    PHANToMTest

Go through the calibration test and make sure that the values are changing.  If everything is working, continue to install the OpenHaptics Toolkit by going to the OpenHaptics\_Linux\_v3\_0/OpenHaptics-AE 3.0/64-bit folder and running

    sudo dpkg -i openhaptics-ae_3.0-2_amd64.deb

After installation, remove duplicate shared libraries from /usr/lib64

    sudo rm /usr/lib64/libPHANToMIO.so /usr/lib64/libPHANToMIO.so.4 /usr/lib64/libPHANToMIO.so.4.3

Create a configuration file in /etc/ld.so.conf.d

    sudo touch /etc/ld.so.conf.d/openhaptics.conf

Add the following line to openhaptics.conf

    /usr/lib64

Rerun ldconfig to find the new shared libraries

    sudo ldconfig

The OpenHaptics Toolkit and Phantom Omni driver should now be installed!  The ROS phantom\_omni and omni\_description packages can be used to interface ROS with the Omni hardware.
