// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.shifts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;

public class AllianceShifts extends SubsystemBase {
  @AutoLogOutput private boolean hubActive = true;
  @AutoLogOutput private Alliance firstAllianceShift;
  @AutoLogOutput private Shifts currentShift = Shifts.AUTONOMOUS;

  private Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  private final Trigger hubActiveTrigger = new Trigger(() -> hubActive);

  public static enum Shifts {
    AUTONOMOUS,
    TRANSITION,
    FIRST,
    SECOND,
    THIRD,
    FOURTH,
    ENDGAME
  }

  private static Command hubInactiveCommand(CommandXboxController controller) {
    return Commands.sequence(
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1.0)),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0)),
        Commands.waitSeconds(0.5));
  }

  /** Creates a new AllianceShifts. */
  public AllianceShifts(CommandXboxController controller) {
    hubActiveTrigger
        .and(() -> DriverStation.isTeleopEnabled())
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 1.0)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> controller.setRumble(RumbleType.kBothRumble, 0.0))));

    hubActiveTrigger.onFalse(
        Commands.sequence(
            hubInactiveCommand(controller),
            hubInactiveCommand(controller),
            hubInactiveCommand(controller)));
  }

  public void checkFirstAllianceShift() {
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          firstAllianceShift = Alliance.Red;
          break;
        case 'R':
          firstAllianceShift = Alliance.Blue;
          break;
        default:
          break;
      }
    }
  }

  public Alliance getFirstAllianceShift() {
    return firstAllianceShift;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isAutonomousEnabled()) // Auto shift
    {
      hubActive = true;
      currentShift = Shifts.AUTONOMOUS;
    } else if (DriverStation.isTeleopEnabled()) {
      if (DriverStation.getMatchTime() >= 130.0) // Transition shift
      {
        hubActive = true;
        currentShift = Shifts.TRANSITION;
      } else if (DriverStation.getMatchTime() >= 105.0) // First shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Blue)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Red);
        currentShift = Shifts.FIRST;
      } else if (DriverStation.getMatchTime() >= 80.0) // Second shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Red)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Blue);
        currentShift = Shifts.SECOND;
      } else if (DriverStation.getMatchTime() >= 55.0) // Third shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Blue)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Red);
        currentShift = Shifts.THIRD;
      } else if (DriverStation.getMatchTime() >= 30.0) // Fourth shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Red)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Blue);
        currentShift = Shifts.FOURTH;
      } else if (DriverStation.getMatchTime() < 30.0) // End game
      {
        hubActive = true;
        currentShift = Shifts.ENDGAME;
      }
    }
  }
}
