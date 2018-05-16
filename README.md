# WaterloopSoftwareChallenge
Code created to simulate Waterloop travel across 30km for the embedded software challenge (https://gist.github.com/dhillondeep/93405873cd7007a90fb848dd582b3259). Has basic functionality without dealing
with error states or server command integration.

Uses Wire library and I2C communication for interfacing between the "pod" Arduino and "master" Arduino. 


Uses Serial to communicate between the "master" Arduino and the computer/server.


Uses low pass filter from Arduino standard library to remove most noise.


Uses double integration using trapezoidal approach for semi-accurate estimation of velocity and distance from acceleration.


*Will be cleaned up to explain and only use one type of method for approximating velocity/distance and flow of the software*
