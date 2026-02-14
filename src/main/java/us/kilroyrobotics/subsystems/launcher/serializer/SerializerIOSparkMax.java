package us.kilroyrobotics.subsystems.launcher.serializer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SerializerIOSparkMax implements SerializerIO {
  private final SparkMax motor;

  public SerializerIOSparkMax(int motorId) {
    this.motor = new SparkMax(motorId, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kCoast);
    motorConfig.smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    inputs.connected = !Double.isNaN(motor.getAppliedOutput());
    inputs.positionRads = Rotations.of(motor.getEncoder().getPosition()).in(Radians);
    inputs.velocityRadsPerSec =
        RotationsPerSecond.of(motor.getEncoder().getVelocity() * 60.0).in(RadiansPerSecond);
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(SerializerIOOutputs outputs) {
    motor.set(outputs.appliedOutput);
  }
}
