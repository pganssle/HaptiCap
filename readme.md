# HaptiCap
The *HaptiCap* is a hat designed to provide directional haptic feedback to the wearer. The prototype
uses 8 vibration motors and is powered by an Arduino Nano.

This repository contains libraries and sketches designed to work with the HaptiCap.

## HaptiCapMag
While the original and primary goal of the *HaptiCap* was to act as a haptic compass (continuously
signaling north), I have not ruled out using the form factor for other directional feedback. For forward compatibility, I've decided to call the general haptic feedback interface (the hat)
*HaptiCap*, and the version that is acting as a haptic compass is called the *HaptiCapMag*.

These sketches assume that your HaptiCap uses an HMC5883L 3-axis digital magnetometer, and that
you are using [my HMC5883L library](https://github.com/pganssle/HMC5883L), which is stored in a
separate repository in case someone wants to use it for non-HaptiCap applications. Full
documentation for my HMC5883L library can be found in that repository or online [here](https://pganssle.github.io/HMC5883L/documentation/).

A full writeup on the device is coming soon.