package us.kilroyrobotics.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class RollerIOSparkMax implements RollerIO {
  private final SparkMax motor;

  /**
   * Creates a new RollerIOSparkMax.
   *
   * @param motorId the CAN ID of the motor
   */
  public RollerIOSparkMax(int motorId) {
    this.motor = new SparkMax(motorId, MotorType.kBrushless);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kCoast);
    motorConfig.smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected = motor.hasStickyFault();
    inputs.positionRads = Rotations.of(motor.getEncoder().getPosition()).in(Radians);
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {
    motor.set(outputs.appliedOutput);
  }
}
