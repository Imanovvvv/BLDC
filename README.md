# BLDC Motor Control System

## Project Description

A Brushless DC (BLDC) motor control system based on the **Field Oriented Control (FOC)** algorithm using the **CAN** protocol.

## Components

- **Motor**: KMTech MG4010E v3
- **Communication Protocol**: CAN (Controller Area Network)
- **Programming Language**: Python 3
- **Libraries**:
  - `python-can` - CAN bus communication
  - `matplotlib` - real-time data visualization

## Installation

```bash
pip install python-can matplotlib
```

## Usage

### Simulation Mode (without hardware)

```bash
python bldc_motor_control.py --simulate
```

### Connect to Real CAN Bus

```bash
# Virtual CAN interface
python bldc_motor_control.py --channel vcan0

# Physical CAN interface
python bldc_motor_control.py --channel can0 --bitrate 500000
```

## Command Line Options

| Option | Short | Default | Description |
|--------|-------|---------|-------------|
| `--channel` | `-c` | `vcan0` | CAN interface (can0, vcan0, usbcan) |
| `--bitrate` | `-b` | `500000` | Bit rate in bits per second |
| `--simulate` | `-s` | `False` | Simulation mode without hardware |

## Features

### BLDCMotorController Class

Main motor control class:

- CAN bus connect/disconnect
- Receive messages from motor
- Parse current data (Id, Iq, Ia, Ib, Ic)
- Send commands to motor
- Generate simulated data for testing

### CurrentVisualizer Class

Real-time current visualization:

- D-Q frame currents display (rotating reference)
- Three-phase A-B-C currents display (stationary reference)
- Real-time metrics calculation and display
- Automatic time axis scaling

## CAN Message Format

### Current Feedback (CAN ID: 0x201)

Expected data format (8 bytes):

| Bytes | Parameter | Type | Scale |
|-------|-----------|------|-------|
| 0-1 | Current Id | int16 (LE) | 0.01 A |
| 2-3 | Current Iq | int16 (LE) | 0.01 A |
| 4-5 | Current Ia | int16 (LE) | 0.01 A |
| 6-7 | Current Ib | int16 (LE) | 0.01 A |

**Note**: Format may vary depending on motor configuration. Please refer to KMTech MG4010E v3 documentation.

## FOC Algorithm

Field Oriented Control (FOC) allows independent control of magnetic flux and torque:

- **Id (Direct current)** - flux-producing current
- **Iq (Quadrature current)** - torque-producing current

Coordinate transformations:
- **ABC → DQ** (Park transform) - stationary to rotating frame
- **DQ → ABC** (Inverse Park transform) - rotating to stationary frame

## CAN Interface Setup on Linux

### Create Virtual CAN Interface for Testing

```bash
# Load vcan module
sudo modprobe vcan

# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Verify interface
ip link show vcan0
```

### Configure Physical CAN Interface

```bash
# Setup CAN interface
sudo ip link set can0 up type can bitrate 500000

# Verify interface
ip -details link show can0
```

## Project Structure

```
/workspace/
├── bldc_motor_control.py    # Main control script
└── README.md                # Documentation
```

## Usage Example

```python
from bldc_motor_control import BLDCMotorController, CurrentVisualizer

# Create controller
controller = BLDCMotorController(channel='can0', bitrate=500000)

# Connect
if controller.connect():
    # Receive message
    msg = controller.receive_message(timeout=1.0)
    
    if msg:
        # Parse current data
        data = controller.parse_current_data(msg)
        print(f"Id: {data['id']:.2f} A, Iq: {data['iq']:.2f} A")
    
    # Disconnect
    controller.disconnect()
```

## Hardware Requirements

- CAN adapter (e.g., USB-CAN, PCAN, SocketCAN compatible)
- KMTech MG4010E v3 motor with CAN interface
- Computer with Linux OS (recommended) or Windows with appropriate drivers

## License

This project is developed for educational purposes.