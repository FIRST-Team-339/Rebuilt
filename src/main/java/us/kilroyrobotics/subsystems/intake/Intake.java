// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants;
import us.kilroyrobotics.Constants.IntakeConstants.ActuatorConstants;
import us.kilroyrobotics.Constants.IntakeConstants.RollerConstants;
import us.kilroyrobotics.Constants.Mode;
import us.kilroyrobotics.subsystems.intake.actuator.Actuator;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIO;
import us.kilroyrobotics.subsystems.intake.roller.Roller;
import us.kilroyrobotics.subsystems.intake.roller.RollerIO;

public class Intake extends SubsystemBase {
  private final Roller roller;
  private final Actuator actuator;

  private IntakeSimulation intakeSimulation = null;

  @AutoLogOutput(key = "Intake/CurrentState")
  private IntakeState currentState = IntakeState.RETRACTED;

  @AutoLogOutput(key = "Intake/PendingEvent")
  private IntakeEvent pendingEvent = IntakeEvent.NONE;

  public Intake(ActuatorIO actuatorIO, RollerIO rollerIO, SwerveDriveSimulation driveSimulation) {
    this(actuatorIO, rollerIO);
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            Inches.of(22),
            Inches.of(11),
            IntakeSimulation.IntakeSide.BACK,
            45);
  }

  /** Creates a new Intake. */
  public Intake(ActuatorIO actuatorIO, RollerIO rollerIO) {
    this.actuator = new Actuator(actuatorIO);
    this.roller = new Roller(rollerIO, actuator::getCurrentPosition);
  }

  private Timer timer = new Timer();
  private boolean agitateBackwards = false;

  @Override
  public void periodic() {
    actuator.periodic();
    roller.periodic();

    double rollerOutput = 0.0;
    Angle actuatorAngle = actuator.getSetPosition();

    switch (currentState) {
      case RETRACTED -> {
        // Do nothing, unless an extend event is being triggered
        if (eventIsTriggered(IntakeEvent.EXTEND)) {
          setState(IntakeState.EXTENDING);
        }
      }

      case EXTENDING -> {
        actuatorAngle = Radians.of(ActuatorConstants.kExtendedRads.get());
        if (actuator.atSetpoint()) {
          setState(IntakeState.EXTENDED);
        }
      }

      case EXTENDED -> {
        // Wait for one of the events to be triggered
        if (eventIsTriggered(IntakeEvent.AGITATE)) {
          setState(IntakeState.AGITATING);
          actuatorAngle = actuatorAngle.minus(Degrees.of(40));
          timer.start();
        } else if (eventIsTriggered(IntakeEvent.RETRACT)) {
          setState(IntakeState.RETRACTING);
        } else if (eventIsTriggered(IntakeEvent.START_INTAKING)) {
          setState(IntakeState.INTAKING);
        }
      }

      case INTAKING -> {
        if (eventIsTriggered(IntakeEvent.STOP_INTAKING)) {
          rollerOutput = 0.0;
          if (Constants.currentMode == Mode.SIM) intakeSimulation.stopIntake();
          setState(IntakeState.EXTENDED);
        } else {
          if (Constants.currentMode == Mode.SIM) intakeSimulation.startIntake();
          rollerOutput = RollerConstants.kIntakePercent.get();
        }
      }

      case RETRACTING -> {
        actuatorAngle = Radians.of(0.0);
        if (actuator.atSetpoint()) {
          setState(IntakeState.RETRACTED);
        }
      }

      case AGITATING -> {
        // Switch to retracting state to finish out if retraction is triggered, or if you are less
        // than 5deg away
        if (eventIsTriggered(IntakeEvent.RETRACT) || actuatorAngle.lt(Degrees.of(5))) {
          rollerOutput = 0.0;
          setState(IntakeState.RETRACTING);
        } else {
          // Semi-oscillate back and forth
          rollerOutput = RollerConstants.kAgitatePercent.get();

          if (agitateBackwards && timer.hasElapsed(1.25)) {
            rollerOutput = RollerConstants.kAgitatePercent.get();
            timer.restart();

            agitateBackwards = false;
            actuatorAngle = actuatorAngle.plus(Degrees.of(35));
          } else if (!agitateBackwards && timer.hasElapsed(0.75)) {
            timer.restart();

            agitateBackwards = true;
            actuatorAngle = actuatorAngle.minus(Degrees.of(40));
          }
        }
      }
    }

    actuator.setPosition(actuatorAngle);
    roller.set(rollerOutput);
  }

  /**
   * Check if an event is pending and should be triggered
   *
   * @param event {@link IntakeEvent}
   * @return if event is triggered/pending
   */
  private boolean eventIsTriggered(IntakeEvent event) {
    if (pendingEvent == event) {
      pendingEvent = IntakeEvent.NONE;
      return true;
    }

    return false;
  }

  /**
   * Get the command to trigger a desired event
   *
   * @param event {@link IntakeEvent}
   * @returna a {@link Command}
   */
  public Command triggerEvent(IntakeEvent event) {
    return runOnce(() -> pendingEvent = event);
  }

  /**
   * Get the pending event
   *
   * @return the currently pending {@link IntakeEvent}
   */
  public IntakeEvent getPendingEvent() {
    return pendingEvent;
  }

  /**
   * Get the current state of the intake
   *
   * @return the current {@link IntakeState}
   */
  public IntakeState getCurrentState() {
    return currentState;
  }

  /**
   * Set the state of the intake
   *
   * @param state {@link IntakeState}
   */
  public void setState(IntakeState state) {
    currentState = state;
  }
}
