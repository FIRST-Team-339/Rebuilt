package us.kilroyrobotics.subsystems.intake.actuator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ActuatorIO {
  @AutoLog
  public static class ActuatorIOInputs {
    public boolean connected;
    public double positionRads;
    public boolean atSetpoint;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public static class ActuatorIOOutputs {
    public double positionRads;

    @AutoLogOutput(key = "Components/Intake/Hopper")
    public Pose3d hopperPose = Pose3d.kZero;

    @AutoLogOutput(key = "Components/Intake/IntakeWalls")
    public Pose3d intakeWallsPose3d =
        new Pose3d(new Translation3d(-0.193, 0.0, 0.2), Rotation3d.kZero);
  }

  default void updateInputs(ActuatorIOInputs inputs) {}

  default void applyOutputs(ActuatorIOOutputs outputs) {}
}
