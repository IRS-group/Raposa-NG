#ROS package idmind_sensorsboard

## Description
This package contains the module for ROS communication with IDMind's Raposa Sensors Board

## Usage
1. Install pyserial package
2. Clone the `idmind_serial2` package
3. Clone this package
4. Run `catkin_make`
5. In `launch/idmind_sensorsboard.launch`, change the board param value to the correct device in your /dev folder
6. Run `roslaunch idmind_sensorsboard idmind_sensorsboard.launch`

##Interfaces
1. Topics Published:
- /idmind_sensors/voltages (Voltages) - Tension and Relay positions information
- /idmind_sensors/lights (Bool) - Current status of Lights
2. Services Provided:
- /idmind_sensors/switch_lights (Trigger) - Switches lights
- /idmind_sensors/switch_motor_relay (Trigger) - Switches motor relay
- /idmind_sensors/switch_cable_relay (Trigger) - Switches cable relay, if cable is connected
- /idmind_sensors/switch_electronic_relay (Trigger) - Switches electronic relay, if cable is connected

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt
