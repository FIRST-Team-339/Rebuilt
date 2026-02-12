// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import us.kilroyrobotics.subsystems.elevator.ElevatorIO.ElevatorIOOutputs;

public class Elevator extends SubsystemBase {
  private static final String name = "Elevator";

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIOOutputs outputs = new ElevatorIOOutputs();

  private Distance position = Meters.of(0.0);
  private boolean isAtSetpoint = false;

  /**
   * Creates a new Elevator.
   *
   * @param io A compatible Elevator IO interface
   */
  public Elevator(ElevatorIO io) {
    this.io = io;

    motorDisconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    isAtSetpoint = inputs.atSetpoint;

    outputs.positionMeters = position.in(Meters);

    io.applyOutputs(outputs);
  }

  /**
   * Get the torque current of the elevator
   *
   * @return torque current in amps
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /**
   * Get the velocity of the elevator
   *
   * @return velocity in Meters/Sec
   */
  public double getVelocity() {
    return inputs.velocityMetersPerSec;
  }

  /**
   * Set the desired position of the elevator
   *
   * @param position the desired position as a {@link Distance}
   */
  public void setPosition(Distance position) {
    this.position = position;
  }

  /**
   * @return the setpoint position as a {@link Distance}
   */
  public Distance getSetPosition() {
    return position;
  }

  /**
   * @return the current position of the elevator as a {@link Distance}
   */
  public Distance getCurrentPosition() {
    return Meters.of(inputs.positionMeters);
  }

  /**
   * @return if the elevator is at the set position
   */
  public boolean atSetpoint() {
    return isAtSetpoint;
  }
}
