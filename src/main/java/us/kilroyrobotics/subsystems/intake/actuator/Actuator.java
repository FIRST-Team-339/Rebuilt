// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.intake.actuator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;

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

    outputs.positionRads = position.in(Radians);

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
}
