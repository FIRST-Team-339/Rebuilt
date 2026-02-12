package us.kilroyrobotics.subsystems.intake.roller;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

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
    public Pose3d pose = new Pose3d(new Translation3d(-0.162, 0.0, 0.593), Rotation3d.kZero);
  }

  default void updateInputs(RollerIOInputs inputs) {}

  default void applyOutputs(RollerIOOutputs outputs) {}
}
