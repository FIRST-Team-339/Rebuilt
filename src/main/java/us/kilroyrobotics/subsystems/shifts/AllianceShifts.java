// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.shifts;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class AllianceShifts extends SubsystemBase {
  @AutoLogOutput private boolean hubActive;
  private Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  @AutoLogOutput private Alliance firstAllianceShift;

  /** Creates a new AllianceShifts. */
  public AllianceShifts() {}

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
          firstAllianceShift = Alliance.Blue;
          break;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isAutonomousEnabled()) // Auto shift
    {
      hubActive = true;
    } else if (DriverStation.isTeleopEnabled()) {
      if (DriverStation.getMatchTime() >= 130.0) // Transition shift
      {
        hubActive = true;
      } else if (DriverStation.getMatchTime() >= 105.0) // First shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Blue)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Red);
      } else if (DriverStation.getMatchTime() >= 80.0) // Second shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Red)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Blue);
      } else if (DriverStation.getMatchTime() >= 55.0) // Third shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Blue)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Red);
      } else if (DriverStation.getMatchTime() >= 30.0) // Fourth shift
      {
        hubActive =
            (firstAllianceShift == Alliance.Blue && alliance == Alliance.Red)
                || (firstAllianceShift == Alliance.Red && alliance == Alliance.Blue);
      } else if (DriverStation.getMatchTime() < 30.0) // End game
      {
        hubActive = true;
      } else {
        hubActive = false;
      }

    } else {
      hubActive = false;
    }
  }
}
