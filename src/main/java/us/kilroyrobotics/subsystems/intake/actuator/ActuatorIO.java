package us.kilroyrobotics.subsystems.intake.actuator;

import org.littletonrobotics.junction.AutoLog;

public interface ActuatorIO {
  @AutoLog
  public static class ActuatorIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class ActuatorIOOutputs {
    public double positionRads;
  }

  default void updateInputs(ActuatorIOInputs inputs) {}

  default void applyOutputs(ActuatorIOOutputs outputs) {}
}
