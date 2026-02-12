# Elevator Subsystem

## Overview

The Elevator subsystem controls a vertical lifting mechanism using 2 NEO brushless motors connected through a 9:1 gear ratio. It provides precise position control for raising and lowering game pieces or robot components.

## Hardware Configuration

### Motors
- **Type**: 2x NEO Brushless Motors (REV Robotics)
- **Gear Ratio**: 9:1 reduction
- **Configuration**: Leader-follower setup
  - Leader Motor CAN ID: 50
  - Follower Motor CAN ID: 51
- **Control**: SparkMax motor controllers with built-in PID
- **Idle Mode**: Brake (holds position when not moving)
- **Current Limit**: 40A per motor

### Mechanical Specifications
- **Drum Radius**: 1 inch (0.0254 meters) - adjustable in Constants
- **Travel Range**: 0.0m to 1.0m (configurable)
- **Carriage Mass**: 5 kg (default, adjustable)

## Software Architecture

The subsystem follows the AdvantageKit IO pattern for hardware abstraction:

```
Elevator (main subsystem)
├── ElevatorIO (interface)
├── ElevatorIOSparkMax (real hardware implementation)
├── ElevatorIOSim (simulation implementation)
└── ElevatorConstants (configuration)
```

### Files
- `Elevator.java` - Main subsystem class
- `ElevatorIO.java` - Hardware abstraction interface
- `ElevatorIOSparkMax.java` - Real hardware implementation
- `ElevatorIOSim.java` - Simulation implementation
- `Constants.java` - Configuration constants

## Control System

### Position Control
The elevator uses closed-loop PID position control:
- **Setpoint Units**: Meters
- **Position Tolerance**: ±2cm (0.02m)
- **Update Rate**: 20ms (50Hz) - standard FRC periodic rate

### PID Tuning
PID constants are fully tunable via NetworkTables:
- **kP = 5.0** - Proportional gain (primary control)
- **kI = 0.0** - Integral gain (steady-state error correction)
- **kD = 0.1** - Derivative gain (damping/smoothing)

See `ELEVATOR_PID_TUNING.md` for detailed tuning instructions.

### Predefined Positions
Three common positions are defined and tunable:
- **Stowed**: 0.0m (starting position)
- **Mid**: 0.5m (intermediate height)
- **High**: 1.0m (maximum extension)

## Usage

### Button Bindings (Default)
- **Left Bumper** → Move to stowed position
- **Right Bumper** → Move to high position
- **Y Button** → Move to mid position

### Programmatic Control

```java
// Get elevator instance
Elevator elevator = ...;

// Set position
import static edu.wpi.first.units.Units.Meters;
elevator.setPosition(Meters.of(0.5)); // Move to 0.5 meters

// Check if at setpoint
if (elevator.atSetpoint()) {
    // Elevator has reached target position
}

// Get current position
Distance currentPos = elevator.getCurrentPosition();

// Get velocity
double velocity = elevator.getVelocity(); // meters/sec

// Get torque current
double current = elevator.getTorqueCurrent(); // amps
```

### Command-Based Usage

```java
// Create command to move to specific position
Command moveToHigh = Commands.runOnce(
    () -> elevator.setPosition(Meters.of(1.0)), 
    elevator
);

// Wait until at position
Command moveAndWait = Commands.sequence(
    Commands.runOnce(() -> elevator.setPosition(Meters.of(0.5)), elevator),
    Commands.waitUntil(elevator::atSetpoint)
);
```

## Data Logging

All elevator data is automatically logged via AdvantageKit:
- Position (meters)
- Velocity (m/s)
- Applied voltage (V)
- Current draw (A) - combined from both motors
- Motor temperature (°C)
- Connection status
- Setpoint reached status

Access logs in AdvantageScope under `/Elevator/`.

## Safety Features

### Motor Protection
- **Current Limiting**: 40A per motor prevents overheating
- **Brake Mode**: Motors actively hold position when stopped
- **Disconnect Detection**: Alert triggered if motor connection lost
- **Temperature Monitoring**: Both motor temps monitored

### Software Limits
- **Min Height**: 0.0m (prevents going below floor)
- **Max Height**: 1.0m (prevents over-extension)
- Limits enforced by simulation (hardware limits should also be used)

### Alerts
- Motor disconnect warning appears on driver station
- Uses debouncing (0.5s) to avoid false alarms

## Simulation

The elevator includes a physics-based simulation using WPILib's `ElevatorSim`:
- Accurate motor modeling (2x NEO characteristics)
- Gravity simulation
- Realistic acceleration and velocity
- Useful for testing control algorithms before hardware

Enable simulation mode in `Constants.java`:
```java
public static final Mode kSimMode = Mode.SIM;
```

## Calibration

### Initial Setup
1. Physically position elevator at bottom (stowed position)
2. Power on robot - encoder automatically zeros on boot
3. Test movement with small increments first
4. Verify mechanical limits match software limits

### Conversion Factors
The elevator uses conversion factors to translate motor rotations to linear position:

```java
// Position: motor rotations → meters
kPositionConversionFactor = (2π × drum_radius) / gear_ratio
                          = (2π × 0.0254m) / 9.0
                          = 0.0177m per motor rotation

// Velocity: motor RPM → m/s  
kVelocityConversionFactor = kPositionConversionFactor / 60
```

Adjust `kDrumRadiusMeters` in Constants if your drum size differs.

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Elevator drifts down | PID too weak or brake mode off | Increase kP, verify brake mode |
| Oscillates at setpoint | PID too aggressive | Decrease kP or increase kD |
| Slow to reach position | PID too conservative | Increase kP |
| Overshoots target | Not enough damping | Increase kD |
| Motor disconnect alert | Loose CAN connection | Check wiring and CAN IDs |
| High current draw | Mechanical binding | Check for obstructions |

## Testing Checklist

- [ ] Verify motor CAN IDs are correct (50, 51)
- [ ] Check mechanical limits prevent over-travel
- [ ] Test at low position first
- [ ] Verify brake mode holds position
- [ ] Tune PID for smooth motion
- [ ] Test all three preset positions
- [ ] Verify current draw is reasonable
- [ ] Check motor temperatures under load
- [ ] Test with expected game piece weight
- [ ] Verify safety limits prevent damage

## Maintenance

### Regular Checks
- Inspect cables and CAN connections
- Check for mechanical wear on lift mechanism
- Verify current draw hasn't increased (indicates wear)
- Clean encoder/motor if exposed to debris
- Re-tune PID if robot mass changes significantly

### Firmware Updates
SparkMax firmware should be kept up to date:
1. Connect via USB using REV Hardware Client
2. Update to latest firmware version
3. Re-verify motor configurations after update

## Advanced Features (Future Enhancements)

### Potential Improvements
1. **Feed-Forward Control**: Add gravity compensation (kG) for better holding
2. **Motion Profiling**: Use trapezoidal profiles for smoother motion
3. **Soft Limits**: Add software-enforced position limits with gradual slowdown
4. **Auto-Zero**: Automatically zero at bottom limit switch
5. **Load Detection**: Detect game piece presence via current draw
6. **Dynamic PID**: Adjust PID based on position/load
7. **Velocity Control**: Add velocity setpoint capability

## References

- WPILib Documentation: https://docs.wpilib.org/
- REV SparkMax Documentation: https://docs.revrobotics.com/
- AdvantageKit: https://github.com/Mechanical-Advantage/AdvantageKit
- PID Tuning Guide: See `ELEVATOR_PID_TUNING.md`

## Version History

- **v1.0** (2026-02-12)
  - Initial implementation
  - 2 NEO motors with 9:1 ratio
  - Position PID control
  - Tunable parameters via NetworkTables
  - Full simulation support
