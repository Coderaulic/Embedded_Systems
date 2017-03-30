# Embedded_Systems

#### Objectives:

- Introduction to the process of specifying and designing embedded systems.
- Lab experiments reinforce fundamental concepts using embedded design and debug tools. 
- Follows the embedded systems development; software and hardware partitioning, processor selection,<br>real-time operating systems, coding in assembly language and C, debugging, and testing.

==============================================================================<br>
Title:&emsp;&emsp;&emsp;&nbsp;&nbsp;&nbsp;
Wireless Rover Navigation<br>
Date:&emsp;&emsp;&emsp;&nbsp;&nbsp;
12/02/2016<br>
Authors:&emsp;&emsp;&nbsp;&nbsp;Ryu Muthui and Robert Griswold<br>
Description:&emsp;Wireless Rover Navigation using XBee Commnication via Arduino
==============================================================================<br>

The idea was to implement our own line following rover (master) and collect data on traversal and navigation information. Once this data is captured, it can send it to the second rover (slave) and have it mimic the master rover’s movement.

The overview of the program is as follows: <br>
The master rover is positioned near the course. The PC will then send a command for it to start. It will then navigate forward until it detects a line and begin to follow the line. While traversing, it will collect navigation data, encode it as 7 byte navigation packet, load it into a 100 byte payload for xbee transmission. When the payload is at least half full, it will attempt to send the payload to the Slave rover, only clearing its buffer on a successful transmission.

The slave rover will also be positioned on an identical course next to the Master Rover. It will accept as many navigation packets as it can, waiting for the PC to send a command to start. After the signal, the Slave rover will synchronize its clock with the first navigation packet, and then execute them in order to mimic the timing and speed of the master rover. The master rover will continue to follow the line until it is lost or it receives a signal from the PC to stop.

See <strong><a href="">documentation</a></strong> for more details on the project.

<strong>Overview:</strong><br>
![overview](https://cloud.githubusercontent.com/assets/10789046/24491848/05836874-14de-11e7-9564-ee0d93cd536b.jpg)

<strong>Fun building these little rovers:</strong>:smiley:<br>
![rovers](https://cloud.githubusercontent.com/assets/10789046/24492440/61e6ecce-14e0-11e7-9a73-e5cc7f57dc30.jpg)

<strong>Logics states for the rovers as finite state machines:</strong><br>
![r1](https://cloud.githubusercontent.com/assets/10789046/24492490/8ddc5a12-14e0-11e7-88b9-afb912f237e9.jpg)
![r2](https://cloud.githubusercontent.com/assets/10789046/24492491/8dddf2aa-14e0-11e7-9b23-b31bfe6bc1b6.jpg)

<strong>Console Display: controls, navigation payload status, and traversal results:</strong><br>
![controls](https://cloud.githubusercontent.com/assets/10789046/24492746/4a30f4ca-14e1-11e7-8096-313871739c50.jpg)
![traversal output](https://cloud.githubusercontent.com/assets/10789046/24492747/4a330a94-14e1-11e7-978a-08fa3c686674.jpg)
![traversal results](https://cloud.githubusercontent.com/assets/10789046/24492748/4a34f78c-14e1-11e7-9f2a-d87f17fef085.jpg)


