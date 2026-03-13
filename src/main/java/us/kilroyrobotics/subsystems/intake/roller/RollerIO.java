package us.kilroyrobotics.subsystems.intake.roller;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants.IntakeConstants.RollerConstants;

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
    public double appliedOutput = 0.0;

    @AutoLogOutput(key = "Components/Intake/Roller")
    public Pose3d pose = new Pose3d(RollerConstants.kRollerTranslation, Rotation3d.kZero);
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void applyOutputs(RollerIOOutputs outputs) {}
}
