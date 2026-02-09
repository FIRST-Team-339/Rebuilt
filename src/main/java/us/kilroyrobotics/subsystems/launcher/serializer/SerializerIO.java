package us.kilroyrobotics.subsystems.launcher.serializer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants.LauncherConstants.SerializerConstants;

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
    public double appliedOutput = 0.0;

    @AutoLogOutput(key = "Components/Launcher/Serializer/Bottom")
    public Pose3d bottomPose = new Pose3d(SerializerConstants.kBottomTranslation, Rotation3d.kZero);

    @AutoLogOutput(key = "Components/Launcher/Serializer/Top")
    public Pose3d topPose = new Pose3d(SerializerConstants.kTopTranslation, Rotation3d.kZero);
  }

  default void updateInputs(SerializerIOInputs inputs) {}

  default void applyOutputs(SerializerIOOutputs outputs) {}
}
