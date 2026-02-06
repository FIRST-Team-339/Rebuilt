package us.kilroyrobotics.subsystems.launcher.serializer;

import org.littletonrobotics.junction.AutoLog;

public interface SerializerIO {
  @AutoLog
  public static class SerializerIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class SerializerIOOutputs {
    public double appliedVoltage = 0.0;
  }

  default void updateInputs(SerializerIOInputs inputs) {}

  default void applyOutputs(SerializerIOOutputs outputs) {}
}
