// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.launcher.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import us.kilroyrobotics.Constants.LauncherConstants.FlywheelConstants;
import us.kilroyrobotics.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputs;

public class Flywheel extends SubsystemBase {
  public static final String name = "Launcher/Flywheel";

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;
  private final Alert followerMotorDisconnected;

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    motorDisconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    followerMotorDisconnected =
        new Alert(name + " follower motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
    followerMotorDisconnected.set(
        !followerMotorConnectedDebouncer.calculate(inputs.followerConnected));

    outputs.velocityRPM = 1000;
    outputs.pose =
        new Pose3d(FlywheelConstants.kTranslation, new Rotation3d(0.0, inputs.positionRads, 0.0));

    io.applyOutputs(outputs);
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
