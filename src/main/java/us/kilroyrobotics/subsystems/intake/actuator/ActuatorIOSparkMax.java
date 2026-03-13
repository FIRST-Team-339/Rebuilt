package us.kilroyrobotics.subsystems.intake.actuator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import us.kilroyrobotics.Constants.IntakeConstants.ActuatorConstants;

public class ActuatorIOSparkMax implements ActuatorIO {
  private final SparkMax motor;

  private final ProfiledPIDController controller;

  private Angle desiredAngle = Radians.of(0.0);

  /**
   * Creates a new AcuatorIOSparkMax.
   *
   * @param motorId the CAN ID of the motor
   */
  public ActuatorIOSparkMax(int motorId) {
    this.motor = new SparkMax(motorId, MotorType.kBrushless);
    this.controller =
        new ProfiledPIDController(
            ActuatorConstants.kP,
            ActuatorConstants.kI,
            ActuatorConstants.kD,
            new Constraints(ActuatorConstants.kMaxVelocity, ActuatorConstants.kMaxAcceleration));
    controller.enableContinuousInput(0.0, 1.0);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.positionWrappingEnabled(true);
    motorConfig.closedLoop.positionWrappingInputRange(0.0, 1.0);
    motorConfig.encoder.positionConversionFactor(1.0 / 5.0);
    motorConfig.absoluteEncoder.positionConversionFactor(1.0);
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(30);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    inputs.connected = !Double.isNaN(motor.getAppliedOutput());
    inputs.positionRads = Units.rotationsToRadians(motor.getAbsoluteEncoder().getPosition());
    inputs.positionRotations = motor.getAbsoluteEncoder().getPosition();
    inputs.atSetpoint = Radians.of(inputs.positionRads).isNear(desiredAngle, 0.15);
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();

    motor.set(controller.calculate(inputs.positionRotations));
  }

  @Override
  public void applyOutputs(ActuatorIOOutputs outputs) {
    desiredAngle = Radians.of(outputs.positionRads);

    controller.setGoal(desiredAngle.in(Rotations));
  }
}
