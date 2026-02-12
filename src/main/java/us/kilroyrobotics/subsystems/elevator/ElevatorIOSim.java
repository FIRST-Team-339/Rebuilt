package us.kilroyrobotics.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import us.kilroyrobotics.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;
  private double appliedVoltage = 0.0;
  private double desiredPosition = 0.0;

  /** Creates a new ElevatorIOSim. */
  public ElevatorIOSim() {
    // Create elevator simulation
    // 2 NEO motors, 9:1 gear ratio
    sim =
        new ElevatorSim(
            DCMotor.getNEO(2), // 2 NEO motors
            ElevatorConstants.kGearing, // 9:1 gear ratio
            ElevatorConstants.kCarriageMassKg, // carriage mass
            ElevatorConstants.kDrumRadiusMeters, // drum radius
            ElevatorConstants.kMinHeightMeters, // min height
            ElevatorConstants.kMaxHeightMeters, // max height
            true, // simulate gravity
            0.0); // starting height
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update simulation
    sim.update(0.02); // 20ms period

    inputs.connected = true;
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.atSetpoint = Math.abs(inputs.positionMeters - desiredPosition) < 0.02;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(ElevatorIOOutputs outputs) {
    desiredPosition = outputs.positionMeters;

    // Simple proportional control for simulation using tunable PID
    double error = desiredPosition - sim.getPositionMeters();
    appliedVoltage = error * ElevatorConstants.kP.get() * 12.0; // Scale by battery voltage
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage)); // Clamp to ±12V

    sim.setInputVoltage(appliedVoltage);
  }
}
