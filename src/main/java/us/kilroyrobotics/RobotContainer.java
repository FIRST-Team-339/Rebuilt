// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package us.kilroyrobotics;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.io.IOException;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import us.kilroyrobotics.Constants.DriveConstants;
import us.kilroyrobotics.Constants.IntakeConstants.ActuatorConstants;
import us.kilroyrobotics.Constants.IntakeConstants.RollerConstants;
import us.kilroyrobotics.Constants.LauncherConstants.FlywheelConstants;
import us.kilroyrobotics.Constants.LauncherConstants.KickerConstants;
import us.kilroyrobotics.Constants.LauncherConstants.SerializerConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.drive.Drive;
import us.kilroyrobotics.subsystems.drive.GyroIO;
import us.kilroyrobotics.subsystems.drive.GyroIOPigeon2;
import us.kilroyrobotics.subsystems.drive.GyroIOSim;
import us.kilroyrobotics.subsystems.drive.ModuleIO;
import us.kilroyrobotics.subsystems.drive.ModuleIOSim;
import us.kilroyrobotics.subsystems.drive.ModuleIOTalonFX;
import us.kilroyrobotics.subsystems.drive.Zone;
import us.kilroyrobotics.subsystems.drive.Zone.ZoneType;
import us.kilroyrobotics.subsystems.intake.Intake;
import us.kilroyrobotics.subsystems.intake.IntakeEvent;
import us.kilroyrobotics.subsystems.intake.IntakeState;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIO;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIOSim;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIOSparkMax;
import us.kilroyrobotics.subsystems.intake.roller.RollerIO;
import us.kilroyrobotics.subsystems.intake.roller.RollerIOSim;
import us.kilroyrobotics.subsystems.intake.roller.RollerIOSparkMax;
import us.kilroyrobotics.subsystems.launcher.Launcher;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIO;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIOSim;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIOSparkMax;
import us.kilroyrobotics.subsystems.launcher.kicker.KickerIO;
import us.kilroyrobotics.subsystems.launcher.kicker.KickerIOSim;
import us.kilroyrobotics.subsystems.launcher.kicker.KickerIOSparkMax;
import us.kilroyrobotics.subsystems.launcher.serializer.SerializerIO;
import us.kilroyrobotics.subsystems.launcher.serializer.SerializerIOSim;
import us.kilroyrobotics.subsystems.launcher.serializer.SerializerIOSparkMax;
import us.kilroyrobotics.subsystems.shifts.AllianceShifts;
import us.kilroyrobotics.subsystems.vision.Vision;
import us.kilroyrobotics.subsystems.vision.VisionIO;
import us.kilroyrobotics.subsystems.vision.VisionIOLimelight;
import us.kilroyrobotics.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final SwerveDriveSimulation driveSimulation;
  private final Launcher launcher;
  public final AllianceShifts allianceShifts;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Intake intake;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);
  public final CommandGenericHID streamdeck = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Alert autoPathInfo =
      new Alert("Selected Autonomous is NOT a PathPlanner auto", AlertType.kInfo);

  // Drive Mode
  @AutoLogOutput(key = "Odometry/AutoRotateEnabled")
  private boolean autoRotate = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        driveSimulation = null;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        allianceShifts = new AllianceShifts(controller);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-fl", drive::getRotation));

        intake =
            new Intake(
                new ActuatorIOSparkMax(ActuatorConstants.kMotorCanId),
                new RollerIOSparkMax(RollerConstants.kMotorCanId));

        launcher =
            new Launcher(
                new SerializerIOSparkMax(SerializerConstants.kMotorCanId),
                new KickerIOSparkMax(KickerConstants.kMotorCanId),
                new FlywheelIOSparkMax(
                    FlywheelConstants.kMotorCanId, FlywheelConstants.kFollowerMotorCanId),
                drive::getChassisSpeeds,
                drive::getPose);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(2.0, 2.0, Rotation2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        allianceShifts = new AllianceShifts(controller);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.camera0SimProperties,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose));

        intake = new Intake(new ActuatorIOSim(), new RollerIOSim(), driveSimulation);

        launcher =
            new Launcher(
                new SerializerIOSim(),
                new KickerIOSim() {},
                new FlywheelIOSim(),
                drive::getChassisSpeeds,
                drive::getPose,
                driveSimulation,
                intake.getIntakeSimulation());
        break;

      default:
        // Replayed robot, disable IO implementations
        driveSimulation = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        allianceShifts = new AllianceShifts(controller);

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        intake = new Intake(new ActuatorIO() {}, new RollerIO() {});

        launcher =
            new Launcher(
                new SerializerIO() {},
                new KickerIO() {},
                new FlywheelIO() {},
                drive::getChassisSpeeds,
                drive::getPose);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization());
    autoChooser.addOption("Drive Simple FF Characterization", drive.feedforwardCharacterization());
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.onChange(
        (Command command) -> {
          if (command != null) {
            if (command.getName().equals("InstantCommand")) {
              drive.displayFullAutoPath(null);
              autoPathInfo.set(true);
            }

            try {
              drive.displayFullAutoPath(
                  PathPlannerAuto.getPathGroupFromAutoFile(command.getName()));
              autoPathInfo.set(false);
            } catch (IOException | ParseException e) {
              drive.displayFullAutoPath(null);
              autoPathInfo.set(true);
            }
          }
        });

    configureButtonBindings();
  }

  private Command cancelAutoRotate() {
    return Commands.runOnce(() -> autoRotate = false);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            drive.joystickDriveAtAngle(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller
        .x()
        .onTrue(Commands.sequence(Commands.runOnce(drive::stopWithX, drive), cancelAutoRotate()));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            drive
                .runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())))
                .ignoringDisable(true));

    controller
        .rightTrigger()
        .onTrue(
            Commands.parallel(
                launcher.spinUpSerializerAndKicker(), intake.triggerEvent(IntakeEvent.AGITATE)))
        .onFalse(
            Commands.parallel(
                launcher.stopSerializerAndKicker(), intake.triggerEvent(IntakeEvent.RETRACT)));
    controller.povDown().onTrue(intake.triggerEvent(IntakeEvent.EXTEND));
    controller.povUp().onTrue(intake.triggerEvent(IntakeEvent.RETRACT));
    controller
        .povRight()
        .onTrue(
            intake.runOnce(
                () -> {
                  if (intake.getCurrentState() == IntakeState.EXTENDED) {
                    CommandScheduler.getInstance()
                        .schedule(intake.triggerEvent(IntakeEvent.START_INTAKING));
                  } else if (intake.getCurrentState() == IntakeState.INTAKING) {
                    CommandScheduler.getInstance()
                        .schedule(intake.triggerEvent(IntakeEvent.STOP_INTAKING));
                  }
                }));

    controller.rightStick().onTrue(Commands.runOnce(() -> autoRotate = !autoRotate));
    controller.button(8).onTrue(cancelAutoRotate());
    controller
        .axisMagnitudeGreaterThan(Axis.kRightX.value, DriveConstants.kAutoRotateCancelThreshold)
        .or(
            controller.axisMagnitudeGreaterThan(
                Axis.kRightY.value, DriveConstants.kAutoRotateCancelThreshold))
        .onTrue(cancelAutoRotate());

    Trigger inAutoRotate = new Trigger(() -> autoRotate);

    Command returnToDefaultDrive =
        Commands.runOnce(
            () -> {
              drive.getDefaultCommand().cancel();

              drive.setDefaultCommand(
                  drive.joystickDrive(
                      () -> -controller.getLeftY(),
                      () -> -controller.getLeftX(),
                      () -> -controller.getRightX()));
            });

    inAutoRotate
        .and(() -> drive.getZoneType() == ZoneType.TRENCH)
        .onTrue(
            Commands.parallel(
                drive.runOnce(
                    () -> {
                      drive.getDefaultCommand().cancel();

                      double currentRotationDeg = drive.getRotation().getDegrees();
                      Rotation2d rotation =
                          (currentRotationDeg <= 0 && currentRotationDeg >= -90)
                                  || (currentRotationDeg >= 0 && currentRotationDeg <= 90)
                              ? Rotation2d.kZero
                              : Rotation2d.k180deg;

                      drive.setDefaultCommand(
                          drive.joystickDriveAtAngle(
                              () -> -controller.getLeftY() * DriveConstants.kTrenchSpeedMultiplier,
                              () -> -controller.getLeftX() * DriveConstants.kTrenchSpeedMultiplier,
                              () -> rotation));
                    }),
                Commands.runOnce(
                    () -> {
                      if (intake.getCurrentState().ordinal() < IntakeState.EXTENDED.ordinal()) {
                        intake.triggerEvent(IntakeEvent.EXTEND);
                      }
                    })));

    inAutoRotate
        .and(() -> drive.getZoneType() == ZoneType.BUMP)
        .onTrue(
            drive.runOnce(
                () -> {
                  drive.getDefaultCommand().cancel();

                  Rotation2d rotation;

                  boolean wasInNeutralZone = drive.getPreviousZone() == Zone.NEUTRAL_ZONE;
                  boolean inAllianceHalf = drive.getPose().getX() < (FlippingUtil.fieldSizeX / 2.0);

                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                    inAllianceHalf = !inAllianceHalf;

                  if ((wasInNeutralZone && inAllianceHalf)
                      || (!wasInNeutralZone && !inAllianceHalf)) {
                    rotation = new Rotation2d(Degrees.of(45));
                  } else {
                    rotation = new Rotation2d(Degrees.of(135));
                  }

                  drive.setDefaultCommand(
                      drive.joystickDriveAtAngle(
                          () -> -controller.getLeftY(),
                          () -> -controller.getLeftX(),
                          () -> rotation));
                }));

    inAutoRotate
        .and(() -> drive.getZoneType() == ZoneType.NORMAL_ALLIANCE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.getDefaultCommand().cancel();

                  drive.setDefaultCommand(
                      drive.joystickDriveAtAngle(
                          () -> -controller.getLeftY(),
                          () -> -controller.getLeftX(),
                          () -> new Rotation2d(launcher.getTargetRotation())));
                },
                launcher));

    inAutoRotate
        .and(() -> drive.getZoneType() == ZoneType.NORMAL_OTHER)
        .onTrue(returnToDefaultDrive);

    inAutoRotate.onFalse(returnToDefaultDrive);

    streamdeck
        .button(1)
        .onTrue(Commands.runOnce(() -> allianceShifts.setFirstAllianceShift(Alliance.Blue)));
    streamdeck
        .button(2)
        .onTrue(Commands.runOnce(() -> allianceShifts.setFirstAllianceShift(Alliance.Red)));
    streamdeck
        .button(3)
        .onTrue(
            Commands.runOnce(
                () -> intake.overrideActuator(Degrees.of(ActuatorConstants.kExtendedDegs.get()))));
    streamdeck.button(4).onTrue(Commands.runOnce(() -> intake.overrideActuator(Radians.of(0.0))));
    streamdeck
        .button(5)
        .onTrue(
            Commands.runOnce(
                () -> intake.overrideRoller(RollerConstants.kIntakePercent.get()), intake));
    streamdeck.button(6).onTrue(Commands.runOnce(() -> intake.overrideRoller(0.0), intake));
    streamdeck
        .button(7)
        .onTrue(
            Commands.runOnce(
                () -> intake.overrideRoller(RollerConstants.kOuttakePercent.get()), intake));
    streamdeck
        .button(8)
        .onTrue(launcher.reverseSerializerAndKicker())
        .onFalse(launcher.stopSerializerAndKicker());
    streamdeck
        .button(9)
        .onTrue(
            launcher.run(
                () -> {
                  launcher.overrideFlywheelRPM(
                      (int)
                          (((streamdeck.getRawAxis(0) + 1.0) / 2.0)
                              * 6784
                              * FlywheelConstants.overrideMultiplier));
                }))
        .onFalse(launcher.cancelFlywheelRPMOverride());

    NamedCommands.registerCommand("Spin Up Serializer and Kicker", launcher.spinUpSerializerAndKicker());
    NamedCommands.registerCommand("Stop Serializer and Kicker", launcher.stopSerializerAndKicker());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
