// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import us.kilroyrobotics.Constants.IntakeConstants;
import us.kilroyrobotics.subsystems.intake.actuator.Actuator;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIO;
import us.kilroyrobotics.subsystems.intake.roller.Roller;
import us.kilroyrobotics.subsystems.intake.roller.RollerIO;

public class Intake extends SubsystemBase {
  private final Roller roller;
  private final Actuator actuator;

  @AutoLogOutput(key = "Intake/CurrentState")
  private IntakeState currentState = IntakeState.RETRACTED;

  @AutoLogOutput(key = "Intake/PendingEvent")
  private IntakeEvent pendingEvent = IntakeEvent.NONE;

  /** Creates a new Intake. */
  public Intake(RollerIO rollerIO, ActuatorIO actuatorIO) {
    this.roller = new Roller(rollerIO);
    this.actuator = new Actuator(actuatorIO);
  }

  @Override
  public void periodic() {
    roller.periodic();
    actuator.periodic();

    double rollerVolts = 0.0;
    Angle actuatorAngle = actuator.getPosition();

    switch (currentState) {
      case RETRACTED -> {
        if (eventIsTriggered(IntakeEvent.EXTEND)) {
          setState(IntakeState.EXTENDING);
        }
      }

      case EXTENDING -> {
        actuatorAngle = Radians.of(IntakeConstants.kActuatorExtendedRads.get());
        if (actuator.atSetpoint()) {
          setState(IntakeState.EXTENDED);
        }
      }

      case EXTENDED -> {
        if (eventIsTriggered(IntakeEvent.AGITATE)) {
          setState(IntakeState.AGITATING);
        } else if (eventIsTriggered(IntakeEvent.RETRACT)) {
          setState(IntakeState.RETRACTING);
        } else if (eventIsTriggered(IntakeEvent.START_INTAKING)) {
          setState(IntakeState.INTAKING);
        }
      }

      case INTAKING -> {
        if (eventIsTriggered(IntakeEvent.STOP_INTAKING)) {
          rollerVolts = 0.0;
          setState(IntakeState.EXTENDED);
        } else {
          rollerVolts = IntakeConstants.kRollerIntakeVolts.get();
        }
      }

      case RETRACTING -> {
        actuatorAngle = Radians.of(0.0);
        if (actuator.atSetpoint()) {
          setState(IntakeState.RETRACTED);
        }
      }

      case AGITATING -> {}
    }

    roller.setVolts(rollerVolts);
    actuator.setPosition(actuatorAngle);
  }

  private boolean eventIsTriggered(IntakeEvent event) {
    if (pendingEvent == event) {
      pendingEvent = IntakeEvent.NONE;
      return true;
    }

    return false;
  }

  public Command triggerEvent(IntakeEvent event) {
    return runOnce(() -> pendingEvent = event);
  }

  public IntakeEvent getPendingEvent() {
    return pendingEvent;
  }

  public IntakeState getCurrentState() {
    return currentState;
  }

  public void setState(IntakeState state) {
    currentState = state;
  }
}
