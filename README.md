# PX4 Motor Failure Injection, Detection and Recovery

This repository contains a customized PX4 simulation environment that enables modeling, detection, and response to **single motor failure** in a quadcopter. The project supports both **Gazebo Classic (ROS1)** and **Gazebo Harmonic (ROS2)** environments.

---

## Important Notes

- This firmware is designed for **Gazebo-Classic (ROS1)** and **Gazebo-Harmonic (ROS2)** simulations.
- **Ignore inbuilt PX4 failsafe messages** that may appear in the PX4 console during simulation.
- If the firmware does not build after unzipping, clone the repo directly using:

  ```bash
  git clone https://github.com/avianbob/PX4-Fault-Tolerent-Control.git
  ```

---

## PX4 Firmware Setup

1. Download or clone the repository.

2. Navigate to the PX4 firmware directory:

3. Initialize the firmware to register the custom `uORB` message (`failure_flag.msg`):

4. Launch the PX4 simulation:

---

## Motor Failure Injection

### Takeoff

Once the simulator is running, takeoff using:

```bash
commander takeoff
```

By default, a **discrete complete shutdown** will occur on **motor index 0** after **30 seconds** of simulation time.

### Configuration File

To implement different types of failure you have to edit the following file - `PX4-Autopilot/src/modules/control_allocator/ControlAllocator.cpp`

### Failure Types

#### 1. Complete Shutdown of Rotor

##### a. Discrete Failure

- Sets motor thrust to zero instantly.

Modify:
- Line 736: `failure_time_seconds`
- Line 762: `motor_idx`

##### b. Gradual Failure

- Decreases RPM linearly to zero.

Modify:
- Line 790: `failure_time_seconds`
- Line 814: `motor_idx`
- Line 786: `BASE_RPM` (starting RPM)
- Line 788: `FAILURE_DURATION`

#### 2. Partial Loss of Effectiveness

##### a. Discrete Failure

- Reduces thrust to a set value between 0 and 1.

Modify:
- Line 736: `failure_time_seconds`
- Line 762: `motor_idx`
- Line 763: `actuator_motors.control[motor_idx] = VALUE`

##### b. Gradual Failure

- Reduces RPM linearly to a target value.

Modify:
- Line 790: `failure_time_seconds`
- Line 814: `motor_idx`
- Line 786: `BASE_RPM`
- Line 788: `FAILURE_DURATION`
- Line 787: `MIN_RPM` (target RPM)

#### 3. Vibrational Failure

- Simulates sinusoidal fluctuations in motor RPM.

Modify:
- Line 853: `failure_time_seconds`
- Line 876: `motor_idx`
- Line 849: `BASE_RPM`
- Line 850: `AMP_RPM` (amplitude)
- Line 851: `FREQ_HZ` (frequency)

---

## Motor Failure Detection

Once airborne (`commander takeoff`), detection logic automatically monitors motor failure.

Edit: `PX4-Autopilot/src/modules/commander/Commander.cpp`

Modify:
- Line 224: `windowSize` – sliding window size for detection
- Line 579: `thres_err` – threshold error for triggering
- Line 597: `alpha` – smoothing filter parameter

Detection will trigger as soon as the configured motor fails.

---

## Control Algorithm Switching

Upon failure detection, the controller switches to a **custom recovery strategy**.

### Available Controllers

1. **Geometric Control with MRAC**  
2. **Sliding Mode Control (SMC)**  
3. **Model Predictive Control (MPC) with INDI**

### Using Custom Controllers

1. Navigate to `Control_Algorithms` folder

2. Replace target files:

#### a. MRAC

```bash
cp ControlAllocator-Geometric.cpp ../PX4-Autopilot/src/modules/control_allocator/ControlAllocator.cpp
```

#### b. SMC

```bash
cp Commander-Sliding.cpp ../PX4-Autopilot/src/modules/commander/Commander.cpp
```

#### c. MPC + INDI

```bash
cp Commander-MPC.cpp ../PX4-Autopilot/src/modules/commander/Commander.cpp
```

3. Rebuild PX4-Autopilot

## Wind Simulation

To add wind effects:

1. Navigate to:

   ```bash
   cd PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
   ```

2. Open the `.world` file and **uncomment** the `wind_plugin` block.

