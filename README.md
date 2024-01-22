# topic_latency_test
## Overview
Package to test the reliability and latency of a ROS system in two ways.
- Test 1. comparing the send and receive times of a message for a given topic
- Test 2. evaluating the time for a /cmd_vel packet sent to be acted upon and that command then observed in /odom

### Details for Test 1.
This test simply records the internal timestamps within the header of a given ROS message and records the local time that the messages were received alongside them. From this we can evaluate the consistency of the output of the data from the source as well as the latency of transmission to our local machine.

This test includes delays due to:
- clock differences between the computers
- network latency
- latency between the header timestamp being populated and any other operations prior to it being sent (populating other fields etc.)
- queuing of messages on the receiver
- note that this list is not exhaustive

This test does not explicitly include delays due to:
- internal data acquisition on the robot's internal computer
- transit between the sensor and the robot's internal computer
- software operations prior to the header timestamp being populated on the robot's computer
- note that this list is not exhaustive

The above inclusions and exclusions can be represented by the following diagram.

```mermaid
graph TD
    subgraph "Sensor Data Journey"
    s1[Physical Change in the Environemnt]

    subgraph "Sensor"
    a1[Analogue Sensing Element Observation]
    a2[Sensor Internal ADC]
    a3[Sensor Internal Processing]
    a4[Sensor Tx Buffer]
    end

    subgraph "Robot's Internal Computer"
    b1[Rx Buffer]
    b2[Internal Processing]
    b3[Tx Buffer Processing]
    end

    subgraph "External ROS Computer"
    c1[Rx Buffer]
    c2[Internal Processing]
    end

    s1--->a1-->a2-->a3-->a4-- Data Transmission --->b1-->b2-->b3-- Data Transmission --->c1-->c2

    end

    p1(ROS Message Timestamped)
    p2(ROS Message Received Time)

    p1-.->b2
    p2-.->c2

    b2==NTP===c2
```

### Details for Test 2.
This test tries to give an idea of the time it takes for a command, /cmd_vel, to be acted upon and observed by the robot in the /odom topic. This is useful for evaluating an lower bound for control loop time and the drive system's performance. There are many better ways to achieve real-time performance with ROS, as such this tool is designed for prototyping and research purposes only, whilst trying to have application across a wide range of mobile robots by utilising the common nav_stack topics.

An extension and simplification of the diagram above is provided below to show the exgtension for the control loop being evaluated. Note that on this test we do not look at the internal message time stamps, rather we record the sent and received times on the "External ROS Computer" running the node provided in this repository, as such this test is less reliant on clock synchronisation.

We record a send time, send a /cmd_vel message to our downstream move base and motor driver interfaces. We then wait to see a motion occur in the /odom topic reported back with an appropriate sign for the velocity. Once we have received this we record a receive time and compute the delta t, flip the sign of the velocity command for the next loop and pause for 2 seconds before repeating.

```mermaid
sequenceDiagram
    participant scene as Environment
    participant mot as Motor
    participant sen as Sensor
    participant rob as Robot's Internal Computer
    participant ext as External ROS Computer

    loop
        Note over ext: Record Send Time
        activate ext
        ext-->>rob: Send Velocity Command as /cmd_vel
        deactivate ext
        activate rob
        rob-->>mot: Motor Command Transmission
        deactivate rob
        activate mot
        mot-->>scene: Motor Enacts Physical Change
        deactivate mot
        activate scene
        scene-->>sen: Sensor Observes Change
        deactivate scene
        activate sen
        sen-->>rob: Sensor Data Transmission
        deactivate sen
        activate rob
        rob-->>ext: Sensor Data Transmission as /odom
        deactivate rob
        activate ext
        deactivate ext
        Note over ext: Record Receive Time of Message With Appropriately Signed Velocity
        Note over ext: Compute Loop Time
        Note over ext: Switch Sign for the Next Velocity Command
        Note over ext: Block for 2 Seconds
    end
```

## Installation
If you intend to run Test 1. I would strongly advise that all system clocks are synchronised using a NTP, GNSS time or similar. I would recommend [Chrony](https://chrony-project.org/). You should probably do this regardless whilst operating a distributed system.

Clone this repository and the iiwa_stack into your catkin_ws/src and build.
```bash
git clone https://github.com/MShields1986/topic_latency_test.git
git clone https://github.com/IFL-CAMP/iiwa_stack # TODO: remove this dep
cd ..
catkin build
```

## Usage
### Test 1.
If you'd like to test locally to start with an /odom publisher is included for convenience
```bash
rosrun topic_latency_test publisher
```

Similarly by default the latency test is setup to listen to /odom but can easily be modified and rebuilt for other topics.
```bash
rosrun topic_latency_test latency_test
```

Latency values will be logged to ROS info and recorded along with the associated timestamps in `data/odom.txt`

### Test 2.
The initial velocity is set to 0.1 m/s and will feasibly be acted upon for 2-3 seconds before a reverse command is sent. As such I'd advise allowing a metre or so in front and behind your robot before commencing with the test. Alternatively you could raise the robot from the ground but then you wouldn't capture any effects due to inertia within the test.

```bash
rosrun topic_latency_test command_latency_test
```

Latency values will be logged to ROS info and recorded along with the associated timestamps in `data/command.txt`

### Results and Processing
Logs are recorded in the `data` directory and there are some helper scripts to generate plots provided in the `scripts` directory.

## Bugs, Issues and Feature Requests
Please report bugs, issues and request features using the [Issue Tracker](https://github.com/MShields1986/topic_latency_test/issues).
