# Findings

1. RoboMuseV robot model is in `mybot_gazebo` and `mybot_description` as the robot named `mybot`.
[Done]2. Delete the malformed STL in `mybot_description` for rviz to work.
3. Links not defined from wheel, lidar to chassis.
4. Model fucked up. Use localisation model.
5. Working in simulation with teleop. Working laserscans. Replace hokuyo with rpLIDAR.
6. Make the robot go straight in Gazebo. Modify Radii of castor in the `.xacro`

# Tips 
Faster X forwarding can be done using

```
ssh -XC -c blowfish-cbc,arcfour ubuntu@<jetson_ip>
```
Enabled Compression. Using faster `blowfish` cipher.

# Options for wireless connectivity
* A full blown hardware like [MI router](https://www.amazon.in/Mi-3C-Router-White/dp/B0719SKK6X?tag=googinhydr18418-21&tag=googinkenshoo-21&ascsubtag=07f4de84-460e-41a4-a69b-5e635f81938e) like this.
	* Requires external power from battery. 
	* Space for mounting.

* A USB wifi adapter like [this](https://www.amazon.in/Edimax-EW-7811UN-Wireless-150M-Adapter/dp/B008TCWPNG?tag=googinhydr18418-21&tag=googinkenshoo-21&ascsubtag=07f4de84-460e-41a4-a69b-5e635f81938e).
	* Low ubuntu support with soft AP(hotspot).
	* Low range and speed.

* PCI-e card like this[Intel card](https://www.amazon.in/Intel-Network-7260-HMWG-R-WiFi-PC/dp/B00MV3N7UO?tag=googinhydr18418-21)
	* Costly (INR 4k)
	* Dangling antennas
	