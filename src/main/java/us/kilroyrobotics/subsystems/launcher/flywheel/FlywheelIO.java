package us.kilroyrobotics.subsystems.launcher.flywheel;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants.LauncherConstants.FlywheelConstants;

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

    @AutoLogOutput(key = "Components/Launcher/Flywheel")
    public Pose3d pose = new Pose3d(FlywheelConstants.kTranslation, Rotation3d.kZero);
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
