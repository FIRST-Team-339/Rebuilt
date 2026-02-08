// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import us.kilroyrobotics.subsystems.intake.roller.RollerIO.RollerIOOutputs;

public class Roller extends SubsystemBase {
  private static final String name = "Intake/Roller";

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  private final RollerIOOutputs outputs = new RollerIOOutputs();

  private final Supplier<Angle> actuatorRadsSupplier;

  private double output = 0.0;

  /** 
   * Creates a new Roller. 
   * 
   * @param io A compatible Actuator IO interface
   * @param actuatorRadsSupplier an {@link Supplier Angle Supplier} for the actuator
   */
  public Roller(RollerIO io, Supplier<Angle> actuatorRadsSupplier) {
    this.io = io;
    this.actuatorRadsSupplier = actuatorRadsSupplier;

    motorDisconnected = new Alert(name + "motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    outputs.appliedOutput = output;
    double actualRadsTheta = actuatorRadsSupplier.get().in(Radians);
    outputs.pose =
        new Pose3d(
            new Translation3d(
                -0.193 + (0.031 * Math.cos(actualRadsTheta)) - (0.393 * Math.sin(actualRadsTheta)),
                0.0,
                0.2 + (0.031 * Math.sin(actualRadsTheta)) + (0.393 * Math.cos(actualRadsTheta))),
            new Rotation3d(0.0, inputs.positionRads, 0.0));

    io.applyOutputs(outputs);
  }

  /**
   * Get the torque current of the actuator
   * @return torque current in amps
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /**
   * Get the velocity of the actuator 
   * @return velocity in Radians/Sec
   */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  /**
   * Set the desired speed of the motor as a percent output
   * @param output percent output (-1.0 to 1.0)
   */
  public void set(double output) {
    this.output = output;
  }

  /**
   * Stop the motor
   */
  public void stop() {
    set(0.0);
  }
}
