# WaterloopSoftwareChallenge
Code created to simulate Waterloop travel across 30000km for the embedded software challenge. Has basic functionality without dealing
with error states or server command integration.

Uses Wire library and I2C communication for interfacing between the "pod" Arduino and "master" Arduino. 


Uses Serial to communicate between the "master" Arduino and the computer/server.


Uses low pass filter from Arduino standard library to remove most noise.


Uses double integration using trapezoidal approach for accurate estimation of velocity and distance from acceleration.
