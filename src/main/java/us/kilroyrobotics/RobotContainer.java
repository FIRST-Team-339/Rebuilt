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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import us.kilroyrobotics.Constants.IntakeConstants.ActuatorConstants;
import us.kilroyrobotics.Constants.IntakeConstants.RollerConstants;
import us.kilroyrobotics.Constants.VisionConstants;
import us.kilroyrobotics.commands.DriveCommands;
import us.kilroyrobotics.generated.TunerConstants;
import us.kilroyrobotics.subsystems.drive.Drive;
import us.kilroyrobotics.subsystems.drive.GyroIO;
import us.kilroyrobotics.subsystems.drive.GyroIOPigeon2;
import us.kilroyrobotics.subsystems.drive.GyroIOSim;
import us.kilroyrobotics.subsystems.drive.ModuleIO;
import us.kilroyrobotics.subsystems.drive.ModuleIOSim;
import us.kilroyrobotics.subsystems.drive.ModuleIOTalonFX;
import us.kilroyrobotics.subsystems.intake.Intake;
import us.kilroyrobotics.subsystems.intake.IntakeEvent;
import us.kilroyrobotics.subsystems.intake.IntakeState;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIO;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIOSim;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIOSparkMax;
import us.kilroyrobotics.subsystems.intake.roller.RollerIO;
import us.kilroyrobotics.subsystems.intake.roller.RollerIOSim;
import us.kilroyrobotics.subsystems.intake.roller.RollerIOSparkMax;
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

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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

        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOLimelight("FL-LL2", drive::getRotation));

        intake =
            new Intake(
                new ActuatorIOSparkMax(ActuatorConstants.kMotorCanId),
                new RollerIOSparkMax(RollerConstants.kMotorCanId));
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.camera0SimProperties,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose));

        intake = new Intake(new ActuatorIOSim(), new RollerIOSim(), driveSimulation);
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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        intake = new Intake(new ActuatorIO() {}, new RollerIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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

    // Configure the button bindings
    configureButtonBindings();
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
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.povDown().onTrue(intake.triggerEvent(IntakeEvent.EXTEND));
    controller.povUp().onTrue(intake.triggerEvent(IntakeEvent.RETRACT));
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intake.getCurrentState() == IntakeState.EXTENDED) {
                    CommandScheduler.getInstance()
                        .schedule(intake.triggerEvent(IntakeEvent.START_INTAKING));
                  } else if (intake.getCurrentState() == IntakeState.INTAKING) {
                    CommandScheduler.getInstance()
                        .schedule(intake.triggerEvent(IntakeEvent.STOP_INTAKING));
                  }
                },
                intake));
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
