// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package us.kilroyrobotics.subsystems.launcher.serializer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import us.kilroyrobotics.Constants.LauncherConstants.SerializerConstants;
import us.kilroyrobotics.subsystems.launcher.serializer.SerializerIO.SerializerIOOutputs;

public class Serializer extends SubsystemBase {
  private static final String name = "Launcher/Serializer";

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();
  private final SerializerIOOutputs outputs = new SerializerIOOutputs();

  private double output = 0.0;

  /** Creates a new Serializer. */
  public Serializer(SerializerIO io) {
    this.io = io;

    motorDisconnected = new Alert(name + "motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));

    outputs.appliedOutput = output;

    var rotation = new Rotation3d(0.0, inputs.positionRads, 0.0);

    outputs.bottomPose = new Pose3d(SerializerConstants.kBottomTranslation, rotation);
    outputs.topPose = new Pose3d(SerializerConstants.kTopTranslation, rotation.times(-1));

    io.applyOutputs(outputs);
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void set(double output) {
    this.output = output;
  }

  public void stop() {
    set(0.0);
  }
}
