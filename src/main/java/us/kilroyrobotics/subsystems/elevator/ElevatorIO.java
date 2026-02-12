package us.kilroyrobotics.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected;
    public double positionMeters;
    public boolean atSetpoint;
    public double velocityMetersPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class ElevatorIOOutputs {
    public double positionMeters;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void applyOutputs(ElevatorIOOutputs outputs) {}
}
