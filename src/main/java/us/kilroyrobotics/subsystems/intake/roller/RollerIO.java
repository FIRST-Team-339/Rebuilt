package us.kilroyrobotics.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class RollerIOOutputs {
    public double appliedVoltage = 0.0;
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void applyOutputs(RollerIOOutputs outputs) {}
}
