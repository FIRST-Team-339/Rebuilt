// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.subsystems.launcher.flywheel.Flywheel;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIO;

public class Launcher extends SubsystemBase {
  private final Flywheel flywheel;

  /** Creates a new Shooter. */
  public Launcher(FlywheelIO flywheelIO) {
    this.flywheel = new Flywheel(flywheelIO);
  }

  @Override
  public void periodic() {
    flywheel.periodic();
  }
}
