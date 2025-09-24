# Used in the future if we ever use complete CANOpen Protocol (Commands not perfect, just for reference)
# Beware of file paths in commands

## Test with fake slave using ros2_canopen repo via VCAN


```bash
# You may have missing dependencies so just install them as needed
colcon build
```
```bash
# On your own pc (not on jetson, you can optionally test with vcan)
sudo modprobe vcan
sudo modprobe can-raw
sudo modprobe can
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
# If no vcan in wsl, set up wsl2 kernel: Search in chatgpt: how to setup WSL2 kernel so that it can use vcan
# Might have issues with vcan with loading missing modules, ask Chatgpt to load needed modules
```

```bash
# or test on the Jetson, with can0 physically connected
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify
ip -details -statistics link show can0

# Result should show something like this

# 4: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
#     link/can  promiscuity 0 minmtu 0 maxmtu 0 
#     can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0 
#           bitrate 1000000 sample-point 0.740 
#           tq 20 prop-seg 18 phase-seg1 18 phase-seg2 13 sjw 1
#           mttcan: tseg1 2..255 tseg2 0..127 sjw 1..127 brp 1..511 brp-inc 1
#           mttcan: dtseg1 1..31 dtseg2 0..15 dsjw 1..15 dbrp 1..15 dbrp-inc 1
#           clock 50000000 
#           re-started bus-errors arbit-lost error-warn error-pass bus-off
#           0          0          0          0          0          0         numtxqueues 1 numrxqueues 1 gso_max_size 65536 gso_max_segs 65535 parentbus platform parentdev c310000.mttcan 
#     RX:  bytes packets errors dropped  missed   mcast           
#              0       0      0       0       0       0 
#     TX:  bytes packets errors dropped carrier collsns           
#              0       0      0       0       0       0 
```

## Ensure Correct Paths

Your paths needs to be correct

Command paths: Ensure files exists when passing an input argument for each command below

In canopen_pixhawk_config/bus.yml: Ensure dcf_path is the correct path that points to canopen_pixhawk_config

## Sanity Check commands that launches two fake slaves and one master
```bash
# Terminal 1: fake slave (node 2)
source install/setup.bash
sudo -E bash -lc 'source install/setup.bash && ros2 launch canopen_fake_slaves basic_slave.launch.py node_id:=2 node_name:=slave_node_1 slave_config:=$(ros2 pkg prefix canopen_tests)/share/canopen_tests/config/simple/simple.eds can_interface_name:=vcan0'

# Terminal 2: fake slave (node 3)
source install/setup.bash
sudo -E bash -lc 'source install/setup.bash && ros2 launch canopen_fake_slaves basic_slave.launch.py node_id:=3 node_name:=slave_node_2 slave_config:=$(ros2 pkg prefix canopen_tests)/share/canopen_tests/config/simple/simple.eds can_interface_name:=vcan0'


# Terminal 3: master (simple bus)
source install/setup.bash
ros2 launch canopen_core canopen.launch.py master_config:=$(ros2 pkg prefix canopen_tests)/share/canopen_tests/config/simple/master.dcf bus_config:=$(ros2 pkg prefix canopen_tests)/share/canopen_tests/config/simple/bus.yml can_interface_name:=vcan0
```

Check ROS: ros2 node list, ros2 topic list

There should be test1 and test2 topics

**Issue of using above code (NO NEED TO FIX): one fake slave crashes when sending messages to ros topic and vise versa because provided simple.eds does not define the object 0x4004**

```bash
# so when I do this:

rodneyshdong@Rd:~/envgo/aquapilot$ ros2 topic pub -1 /test2/proxy_device_2/tpdo canopen_interfaces/msg/COData "{index: 0x4000, subindex: 0, data: 42}"
publisher: beginning loop
publishing #1: canopen_interfaces.msg.COData(index=16384, subindex=0, data=42)

# can dump:

rodneyshdong@Rd:~/envgo/aquapilot$ candump vcan0 | cat
  vcan0  203   [4]  2A 00 00 00
  vcan0  183   [4]  2A 00 00 00

# this ends:
rodneyshdong@Rd:~/envgo/aquapilot$ sudo -E bash -lc 'source install/setup.bash && ros2 launch canopen_fake_slaves basic_slave.launch.py node_id:=3 node_name:=slave_node_2 slave_config:=$(ros2 pkg prefix canopen_tests)/share/canopen_tests/config/simple/simple.eds can_interface_name:=vcan0'
[INFO] [launch]: All log files can be found below /home/rodneyshdong/.ros/log/2025-09-15-14-53-09-515438-Rd-130668
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [basic_slave_node-1]: process started with pid [130706]
[basic_slave_node-1] [INFO] [1757962389.852687489] [slave_node_2]: Reaching inactive state.
[INFO] [launch.user]: node 'basic_slave_node' reached the 'inactive' state, 'activating'.
[basic_slave_node-1] [INFO] [1757962389.855135599] [slave_node_2]: Reaching active state.
[basic_slave_node-1] [INFO] [1757962389.856841230] [slave_node_2]: Created slave for node_id 3.
[basic_slave_node-1] terminate called after throwing an instance of 'lely::canopen::SdoError'
[basic_slave_node-1]   what():  Set:03:4004:00: Object does not exist in the object dictionary (06020000): Object does not exist in the object dictionary
[ERROR] [basic_slave_node-1]: process has died [pid 130706, exit code -6, cmd '/home/rodneyshdong/envgo/aquapilot/install/canopen_fake_slaves/lib/canopen_fake_slaves/basic_slave_node --ros-args -r __node:=slave_node_2 -r __ns:=/ --params-file /tmp/launch_params_9n4bx7vj'].
rodneyshdong@Rd:~/envgo/aquapilot$ 

# and this stays:

rodneyshdong@Rd:~/envgo/aquapilot$ sudo -E bash -lc 'source install/setup.bash && ros2 launch canopen_fake_slaves basic_slave.launch.py node_id:=2 node_name:=slave_node_1 slave_config:=$(ros2 pkg prefix canopen_tests)/share/canopen_tests/config/simple/simple.eds can_interface_name:=vcan0'
[INFO] [launch]: All log files can be found below /home/rodneyshdong/.ros/log/2025-09-15-14-52-43-302625-Rd-130149
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [basic_slave_node-1]: process started with pid [130187]
[basic_slave_node-1] [INFO] [1757962363.857910928] [slave_node_1]: Reaching inactive state.
[INFO] [launch.user]: node 'basic_slave_node' reached the 'inactive' state, 'activating'.
[basic_slave_node-1] [INFO] [1757962363.860187152] [slave_node_1]: Reaching active state.
[basic_slave_node-1] [INFO] [1757962363.862008542] [slave_node_1]: Created slave for node_id 2.

```

## Using custom files in canopen_pixhawk_config (one slave, one master) using VCAN on local PC
- Again, ensure files paths match your file path
```bash
# In one terminal
source install/setup.bash
sudo -E bash -lc 'source install/setup.bash && ros2 launch canopen_fake_slaves basic_slave.launch.py node_id:=2 node_name:=pixhawk_slave slave_config:=/home/rodneyshdong/envgo/ros2_canopen/canopen_pixhawk_config/simple.eds can_interface_name:=vcan0'
```

```bash
# In another terminal
source install/setup.bash
ros2 launch canopen_core canopen.launch.py master_config:=/home/rodneyshdong/envgo/ros2_canopen/canopen_pixhawk_config/master.dcf bus_config:=/home/rodneyshdong/envgo/ros2_canopen/canopen_pixhawk_config/bus.yml can_interface_name:=vcan0
```

**IMPORTANT CONECEPT:**
TPDO: Transmit Process Data Object
RPDO: Receive Process Data Object

CAN TPDO -> ROS /rpdo topic

ROS /tpdo topic -> CAN RPDO

**CANopen Communication Object Identifier Convention:**

The identifier literally is just the prefix of the message. Ex: 182#0100000000000000 -> 182#

CANoepn TPDO1 is 0x180 + NodeID -> If we are using Node 2, then it's 0x182 = 182#. Therefore all messages that is sent using TPDO1 starts with 182#

Similarly for RPDO1 it's 0x200 + NodeID, therefore all message received via RPDO1 starts with 202# (Assuming NodeID = 2)

You can verify using procedure below


**Verify that CAN to ROS works**:

```bash
# In another terminal
# You should see the /pixhawk/pixhawk/rpdo & /pixhawk/pixhawk/tpdo topic
ros2 topic list
source install/setup.bash
ros2 topic echo /pixhawk/pixhawk/rpdo
```

```bash
# In different terminal than above candump command
# Sends a random can message via TPDO1
cansend vcan0 182#0100000000000000
```

GOOD if you can see a CAN message being sent to the ROS topic

**Verify that ROS to CAN works**:

```bash
# Monitor can messages in one terminal received via RPDO1
candump vcan0 | cat
```
```bash
# Publish ROS message in another terminal
source install/setup.bash
ros2 topic pub -1 /pixhawk/pixhawk/tpdo canopen_interfaces/msg/COData "{index: 0x4000, subindex: 0, data: 42}"
```

GOOD if you can see a ROS message being sent to candump

# Test Via  Actual 

## Important Notes

```bash
# Configure can1 with 500k bitrate
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify the configuration
ip -details -statistics link show can0
```
```bash
# Configure can1 with 500k bitrate
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up

# Verify the configuration
ip -details -statistics link show can1
```

In simple.eds: search up transmission type: Default value =
- 0xFF = Asynchronous transmission (sends immediately when data changes)
- 0x01 = Synchronous transmission (only sends when SYNC message received)
- 0x00 = No transmission (manual trigger only)

polling boolean in bus.yml apparantly also changes if it sends once or periodically

## Receiving messages via can test bench and in general

for current dcf it only sends like this: 
```
# Sees in can test bench
cansend slcan0 202#FFFF7D0027101388

# Does not see in can test bench but sees in candump
ros2 topic pub -1 /ros2canopen/pixhawk/tpdo canopen_interfaces/msg/COData "{index: 0x4000, subindex: 0, data: 65535}"
```