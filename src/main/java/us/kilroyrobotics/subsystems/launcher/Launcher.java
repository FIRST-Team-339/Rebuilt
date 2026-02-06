// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.launcher;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants.FieldConstants;
import us.kilroyrobotics.subsystems.launcher.flywheel.Flywheel;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIO;
import us.kilroyrobotics.subsystems.launcher.kicker.Kicker;
import us.kilroyrobotics.subsystems.launcher.kicker.KickerIO;
import us.kilroyrobotics.subsystems.launcher.serializer.Serializer;
import us.kilroyrobotics.subsystems.launcher.serializer.SerializerIO;
import us.kilroyrobotics.util.LoggedTunableNumber;

public class Launcher extends SubsystemBase {
  private static final LoggedTunableNumber serializerIntakeVolts =
      new LoggedTunableNumber("Launcher/Serializer/IntakeVolts", 12.0);
  private static final LoggedTunableNumber kickerIntakeVolts =
      new LoggedTunableNumber("Launcher/Kicker/IntakeVolts", 12.0);

  private final Serializer serializer;
  private final Kicker kicker;
  private final Flywheel flywheel;

  private Supplier<Pose2d> robotPoseSupplier;

  private boolean serializerOn = false;
  private boolean kickerOn = false;

  /** Creates a new Shooter. */
  public Launcher(
      SerializerIO serializerIO,
      KickerIO kickerIO,
      FlywheelIO flywheelIO,
      Supplier<Pose2d> robotPoseSupplier) {
    this.serializer = new Serializer(serializerIO);
    this.kicker = new Kicker(kickerIO);
    this.flywheel = new Flywheel(flywheelIO);

    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void periodic() {
    serializer.periodic();
    kicker.periodic();
    flywheel.periodic();

    if (serializerOn) {
      serializer.setVolts(serializerIntakeVolts.get());
    } else {
      serializer.stop();
    }

    if (kickerOn) {
      kicker.setVolts(kickerIntakeVolts.get());
    } else {
      kicker.stop();
    }
  }

  @AutoLogOutput(key = "Launcher/TargetRotation")
  public Angle getTargetRotation() {
    return Radians.of(
        Math.atan2(
            FieldConstants.hubPose.getY() - robotPoseSupplier.get().getY(),
            FieldConstants.hubPose.getX() - robotPoseSupplier.get().getX()));
  }

  @AutoLogOutput(key = "Test/Pose")
  public final Pose3d test = new Pose3d(FieldConstants.hubPose);

  public Command spinUpSerializerAndKicker =
      runOnce(() -> kickerOn = true)
          .andThen(Commands.waitSeconds(0.2))
          .andThen(runOnce(() -> serializerOn = true));

  public Command stopSerializerAndKicker =
      runOnce(() -> serializerOn = false)
          .andThen(Commands.waitSeconds(0.2))
          .andThen(runOnce(() -> kickerOn = false));
}
