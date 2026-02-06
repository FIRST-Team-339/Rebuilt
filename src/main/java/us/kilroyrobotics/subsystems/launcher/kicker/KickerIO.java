package us.kilroyrobotics.subsystems.launcher.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class KickerIOOutputs {
    public double appliedVoltage = 0.0;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void applyOutputs(KickerIOOutputs outputs) {}
}
