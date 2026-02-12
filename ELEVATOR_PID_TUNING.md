# Elevator PID Tuning Guide

## System Specifications
- **Motors**: 2x NEO motors (brushless)
- **Gear Ratio**: 9:1
- **Control**: Position-based PID control
- **Encoder**: Built-in NEO encoder (through SparkMax)

## Initial PID Values (in Constants.java)

The following PID values have been configured as tunable parameters:

```java
kP = 5.0   // Proportional gain
kI = 0.0   // Integral gain  
kD = 0.1   // Derivative gain
```

## PID Tuning Methodology

### Understanding Each Gain:

1. **kP (Proportional)**: 
   - Controls the immediate response to position error
   - Higher values = faster response but more overshoot
   - Lower values = slower, smoother response
   - **Starting value: 5.0** is a good baseline for elevator systems

2. **kI (Integral)**:
   - Eliminates steady-state error over time
   - Usually not needed for well-tuned elevator systems
   - Can cause instability if too high
   - **Starting value: 0.0** (disabled initially)

3. **kD (Derivative)**:
   - Dampens oscillations and overshoot
   - Helps system settle faster
   - **Starting value: 0.1** provides light damping

### Tuning Process:

#### Step 1: Start with P-only control
1. Set kP = 1.0, kI = 0.0, kD = 0.0
2. Command the elevator to a mid-height position
3. Observe the response:
   - If too slow/sluggish: Increase kP
   - If oscillating: Decrease kP
4. Increase kP until you get slight overshoot (~5-10%)

#### Step 2: Add D gain
1. Keep kP from Step 1
2. Add small amount of kD (start with 0.1)
3. Observe the response:
   - D gain should reduce overshoot and oscillations
   - If system becomes sluggish: Reduce kD
   - If still oscillating: Increase kD
4. Tune until minimal overshoot with fast settling time

#### Step 3: Add I gain (if needed)
1. Only add if you observe steady-state error
2. Start with very small kI (0.01 or less)
3. Gradually increase until error is eliminated
4. **Warning**: Too much I gain causes instability!

### Recommended PID Values by System Type:

#### Light Load (< 5 kg carriage):
```
kP = 5.0 - 8.0
kI = 0.0 - 0.01
kD = 0.1 - 0.3
```

#### Medium Load (5-10 kg carriage):
```
kP = 3.0 - 5.0
kI = 0.0
kD = 0.1 - 0.2
```

#### Heavy Load (> 10 kg carriage):
```
kP = 2.0 - 4.0
kI = 0.0 - 0.02
kD = 0.05 - 0.15
```

## Using Tunable Parameters

All PID values are configured as `LoggedTunableNumber` which means you can tune them live:

1. **During Testing**:
   - Open NetworkTables or AdvantageScope
   - Navigate to `/Elevator/kP`, `/Elevator/kI`, `/Elevator/kD`
   - Adjust values in real-time
   - Observe the elevator response

2. **After Tuning**:
   - Once you find good values, update them in `Constants.java`
   - This ensures they persist across robot reboots

## Safety Considerations

1. **Current Limiting**: Motors are limited to 40A each
2. **Soft Limits**: Configure kMinHeightMeters and kMaxHeightMeters
3. **Brake Mode**: Motors use brake mode to hold position
4. **Watchdog**: Motor disconnect alert will trigger if connection is lost

## Testing Protocol

1. **Static Hold Test**:
   - Command elevator to mid position
   - Verify it holds steady with no drift
   - Check current draw (should be reasonable)

2. **Step Response Test**:
   - Command from low to high position
   - Measure rise time, overshoot, settling time
   - Target: < 5% overshoot, < 1s settling time

3. **Tracking Test**:
   - Command multiple positions in sequence
   - Verify smooth transitions
   - Check for oscillations at setpoint

4. **Load Test**:
   - Repeat tests with expected game piece load
   - May need to increase kP slightly with load

## Advanced Tuning Notes

### Feed-Forward (Optional Enhancement):
For better performance, consider adding feed-forward terms:
- **kG**: Gravity compensation (constant voltage to hold position)
- **kV**: Velocity feed-forward
- **kA**: Acceleration feed-forward

These can be added to the SparkMax configuration if needed.

### Motion Profiling (Optional Enhancement):
Instead of step inputs, use trapezoidal or S-curve motion profiles:
- Reduces mechanical stress
- Smoother motion
- Better control of acceleration/deceleration

## Monitoring and Diagnostics

Key metrics to log:
- Position error (setpoint - actual)
- Velocity
- Current draw (both motors)
- Temperature
- Applied voltage
- Time to reach setpoint

## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Slow response | kP too low | Increase kP |
| Oscillation | kP too high or kD too low | Decrease kP or increase kD |
| Overshoot | kP too high | Decrease kP or increase kD |
| Steady-state error | Friction/gravity not compensated | Add small kI or feed-forward |
| Jerky motion | kD too high | Decrease kD |
| Drifting down | Insufficient kP or no kG | Increase kP or add gravity FF |

## Current Configuration Summary

### Hardware Setup:
- Leader Motor CAN ID: 50
- Follower Motor CAN ID: 51
- Follower configured to mirror leader
- Position control via SparkMax PID controller
- Encoder: NEO built-in encoder with conversion factors

### Software Setup:
- Control loop: 20ms (50Hz) - standard FRC periodic rate
- Position units: Meters
- Setpoint tolerance: ±2cm (0.02m)
- Button bindings:
  - Left Bumper: Stowed position (0.0m)
  - Right Bumper: High position (1.0m)
  - Y Button: Mid position (0.5m)

## Final Tuning Recommendations

Based on a typical FRC elevator with 2 NEOs at 9:1 ratio:

**Recommended Starting Values:**
```java
kP = 5.0   // Aggressive enough for good response
kI = 0.0   // Disabled, not typically needed
kD = 0.1   // Light damping to reduce overshoot
```

**Fine-tuning process:**
1. Test with current values
2. If too aggressive (oscillating): Reduce kP to 3.0, increase kD to 0.2
3. If too sluggish: Increase kP to 7.0
4. Adjust kD based on overshoot (0.05 - 0.3 range typical)

The values are intentionally conservative to ensure safe initial testing. You can be more aggressive once you verify the mechanical system is working correctly.
