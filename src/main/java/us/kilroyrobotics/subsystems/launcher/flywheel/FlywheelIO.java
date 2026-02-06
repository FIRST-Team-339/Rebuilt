package us.kilroyrobotics.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRPM;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;

    public boolean followerConnected;
    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;
  }

  public static class FlywheelIOOutputs {
    public double velocityRPM = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
