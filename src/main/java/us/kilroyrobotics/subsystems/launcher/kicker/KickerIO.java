package us.kilroyrobotics.subsystems.launcher.kicker;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants.LauncherConstants.KickerConstants;

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
    public double appliedOutput = 0.0;

    @AutoLogOutput(key = "Components/Launcher/Kicker/Bottom")
    public Pose3d bottomPose = new Pose3d(KickerConstants.kBottomTranslation, Rotation3d.kZero);

    @AutoLogOutput(key = "Components/Launcher/Kicker/Middle")
    public Pose3d middlePose = new Pose3d(KickerConstants.kMiddleTranslation, Rotation3d.kZero);

    @AutoLogOutput(key = "Components/Launcher/Kicker/Top")
    public Pose3d topPose = new Pose3d(KickerConstants.kTopTranslation, Rotation3d.kZero);
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void applyOutputs(KickerIOOutputs outputs) {}
}
