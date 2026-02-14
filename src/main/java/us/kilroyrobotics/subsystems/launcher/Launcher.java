// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.AutoLogOutput;

import us.kilroyrobotics.Constants;
import us.kilroyrobotics.Constants.FieldConstants;
import us.kilroyrobotics.Constants.LauncherConstants;
import us.kilroyrobotics.Constants.LauncherConstants.FlywheelConstants;
import us.kilroyrobotics.Constants.LauncherConstants.KickerConstants;
import us.kilroyrobotics.Constants.LauncherConstants.SerializerConstants;
import us.kilroyrobotics.Constants.Mode;
import us.kilroyrobotics.subsystems.launcher.flywheel.Flywheel;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIO;
import us.kilroyrobotics.subsystems.launcher.kicker.Kicker;
import us.kilroyrobotics.subsystems.launcher.kicker.KickerIO;
import us.kilroyrobotics.subsystems.launcher.serializer.Serializer;
import us.kilroyrobotics.subsystems.launcher.serializer.SerializerIO;

public class Launcher extends SubsystemBase {
  private final Serializer serializer;
  private final Kicker kicker;
  private final Flywheel flywheel;

  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private Supplier<Pose2d> robotPoseSupplier;

  private SwerveDriveSimulation driveSimulation;
  private IntakeSimulation intakeSimulation;
  private Debouncer launchBallDebouncerSimulation;

  private boolean serializerOn = false;
  private boolean kickerOn = false;

  public Launcher(
      SerializerIO serializerIO,
      KickerIO kickerIO,
      FlywheelIO flywheelIO,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      SwerveDriveSimulation driveSimulation,
      IntakeSimulation intakeSimulation) {
    this(serializerIO, kickerIO, flywheelIO, chassisSpeedsSupplier, robotPoseSupplier);

    this.driveSimulation = driveSimulation;
    this.intakeSimulation = intakeSimulation;
    this.launchBallDebouncerSimulation = new Debouncer(0.1, Debouncer.DebounceType.kFalling);
  }

  /** Creates a new Launcher. */
  public Launcher(
      SerializerIO serializerIO,
      KickerIO kickerIO,
      FlywheelIO flywheelIO,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Pose2d> robotPoseSupplier) {
    this.serializer = new Serializer(serializerIO);
    this.kicker = new Kicker(kickerIO);
    this.flywheel = new Flywheel(flywheelIO);

    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.robotPoseSupplier = robotPoseSupplier;
  }

  /**
   * Calculates the Initial Velocity of the Game Piece Projectile in the X-Y Plane.
   *
   * <p>This method calculates the initial velocity of the game piece projectile, accounting for the
   * chassis's translational and rotational motion as well as the shooter's ground speed.
   *
   * @param groundSpeedMPS the ground component of the projectile's initial velocity, provided as a
   *     scalar in meters per second (m/s)
   * @return the calculated initial velocity of the projectile as a {@link Translation2d} in meters
   *     per second
   */
  private Translation2d calculateInitialProjectileVelocityMPS() {
    final Translation2d
        chassisTranslationalVelocity =
            new Translation2d(
                chassisSpeedsSupplier.get().vxMetersPerSecond,
                chassisSpeedsSupplier.get().vyMetersPerSecond),
        shooterGroundVelocityDueToChassisRotation =
            FlywheelConstants.kTranslation
                .toTranslation2d()
                .rotateBy(robotPoseSupplier.get().getRotation())
                .rotateBy(Rotation2d.fromDegrees(90))
                .times(chassisSpeedsSupplier.get().omegaRadiansPerSecond),
        shooterGroundVelocity =
            chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

    return shooterGroundVelocity.plus(
        new Translation2d(
            flywheel.getVelocity().in(MetersPerSecond)
                * Math.cos(LauncherConstants.kLaunchAngle.in(Radians)),
            robotPoseSupplier.get().getRotation()));
  }

  /**
   * Calculates the Projectile's Position at a Given Time.
   *
   * <p>This method calculates the position of the projectile using the physics formula for
   * projectile motion.
   *
   * @param t the time elapsed after the launch of the projectile, in seconds
   * @return the calculated position of the projectile at time <code>t</code> as a {@link
   *     Translation3d} object
   */
  private Translation3d getPositionAtTime(double t) {
    final double height =
        LauncherConstants.kInitialBallHeight.in(Meters)
            + flywheel.getVelocity().in(MetersPerSecond) * t
            - 1.0 / 2.0 * (11) * t * t;

    final Translation2d current2dPosition =
        robotPoseSupplier.get().getTranslation().plus(FlywheelConstants.kTranslation.toTranslation2d()).rotateBy(robotPoseSupplier.get().getRotation())
            .plus(calculateInitialProjectileVelocityMPS().times(t));
    return new Translation3d(current2dPosition.getX(), current2dPosition.getY(), height);
  }

  @Override
  public void periodic() {
    serializer.periodic();
    kicker.periodic();
    flywheel.periodic();

    if (serializerOn) {
      serializer.set(SerializerConstants.kSerializerPercent.get());
    } else {
      serializer.stop();
    }

    if (kickerOn) {
      kicker.set(KickerConstants.kKickerPercent.get());

      if (Constants.currentMode == Mode.SIM
          && intakeSimulation.obtainGamePieceFromIntake()
          && launchBallDebouncerSimulation.calculate(true)) {

        SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                    driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    FlywheelConstants.kTranslation.toTranslation2d(), // shooter offet from center
                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                    LauncherConstants.kInitialBallHeight, // initial height of the ball, in meters
                    flywheel.getVelocity(), // initial velocity, in m/s
                    LauncherConstants.kLaunchAngle));
      }
    } else {
      kicker.stop();
    }
  }

  @AutoLogOutput(key = "Launcher/TargetRotation")
  public Angle getTargetRotation() {
    return Radians.of(
        Math.atan2(
            FieldConstants.getHubPose().getY() - robotPoseSupplier.get().getY(),
            FieldConstants.getHubPose().getX() - robotPoseSupplier.get().getX()));
  }

  @AutoLogOutput(key = "Launcher/DistanceFromHub")
  public Distance getDistanceFromHub() {
    return Meters.of(
        FieldConstants.getHubPose()
            .getTranslation()
            .getDistance(robotPoseSupplier.get().getTranslation()));
  }

  @AutoLogOutput(key = "Launcher/Trajectory")
  public Pose3d[] getTrajectory() {
    final int maxIterations = 100;
    final double stepSeconds = 0.02;
    List<Pose3d> trajectoryPoints = new ArrayList<>();

    for (int i = 0; i < maxIterations; i++) {
      final double t = i * stepSeconds;
      final Translation3d currentPosition = getPositionAtTime(t);
      trajectoryPoints.add(
          new Pose3d(
              currentPosition,
              new Rotation3d(
                  0,
                  -LauncherConstants.kLaunchAngle.in(Radians),
                  robotPoseSupplier.get().getRotation().getRadians())));

      if (currentPosition.getZ() < (Units.inchesToMeters(3))
          && t * (11)
              > (flywheel.getVelocity().in(MetersPerSecond)
                  * Math.sin(LauncherConstants.kLaunchAngle.in(Radians)))) break;
    }

    return trajectoryPoints.toArray(Pose3d[]::new);
  }

  public Command spinUpSerializerAndKicker =
      runOnce(() -> kickerOn = true)
          .andThen(Commands.waitSeconds(0.2))
          .andThen(runOnce(() -> serializerOn = true));

  public Command stopSerializerAndKicker =
      runOnce(() -> serializerOn = false)
          .andThen(Commands.waitSeconds(0.2))
          .andThen(runOnce(() -> kickerOn = false));
}
