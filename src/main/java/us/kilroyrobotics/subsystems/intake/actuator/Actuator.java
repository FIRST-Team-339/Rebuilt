// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.intake.actuator;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import us.kilroyrobotics.subsystems.intake.actuator.ActuatorIO.ActuatorIOOutputs;

public class Actuator extends SubsystemBase {
  private static final String name = "Intake/Actuator";

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  private final ActuatorIO io;
  private final ActuatorIOInputsAutoLogged inputs = new ActuatorIOInputsAutoLogged();
  private final ActuatorIOOutputs outputs = new ActuatorIOOutputs();

  private Angle position = Radians.of(0.0);
  private boolean isAtSetpoint = false;

  /** Creates a new Actuator. */
  public Actuator(ActuatorIO io) {
    this.io = io;

    motorDisconnected = new Alert(name + "motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    isAtSetpoint = inputs.atSetpoint;

    outputs.positionRads = position.in(Radians);
    outputs.hopperPose =
        new Pose3d(
            new Translation3d(
                (-Math.cos(inputs.positionRads - Units.degreesToRadians(90)) * 0.311658), 0.0, 0.0),
            Rotation3d.kZero);
    outputs.intakeWallsPose3d =
        new Pose3d(
            new Translation3d(-0.193, 0.0, 0.2), new Rotation3d(0.0, -inputs.positionRads, 0.0));

    io.applyOutputs(outputs);
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void setPosition(Angle position) {
    this.position = position;
  }

  public Angle getSetPosition() {
    return position;
  }

  public Angle getCurrentPosition() {
    return Radians.of(inputs.positionRads);
  }

  public boolean atSetpoint() {
    return isAtSetpoint;
  }
}
