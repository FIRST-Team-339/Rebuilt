package us.kilroyrobotics.subsystems.intake.actuator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import us.kilroyrobotics.Constants.IntakeConstants.ActuatorConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ActuatorIOSparkMax implements ActuatorIO {
  private final SparkMax motor;

  private final SparkClosedLoopController controller;

  private Angle desiredAngle = Radians.of(0.0);

  /** 
   * Creates a new AcuatorIOSparkMax.
   * 
   * @param motorId the CAN ID of the motor
   * */
  public ActuatorIOSparkMax(int motorId) {
    this.motor = new SparkMax(motorId, MotorType.kBrushless);
    this.controller = motor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(ActuatorConstants.kP, ActuatorConstants.kI, ActuatorConstants.kD, 0);
    motorConfig.closedLoop.positionWrappingEnabled(true);
    motorConfig.closedLoop.positionWrappingInputRange(0.0, 0.3);
    motorConfig.idleMode(IdleMode.kCoast);
    motorConfig.smartCurrentLimit(40);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    inputs.connected = motor.hasStickyFault();
    inputs.positionRads = Units.rotationsToRadians(motor.getAbsoluteEncoder().getPosition());
    inputs.atSetpoint = Radians.of(inputs.positionRads).isNear(desiredAngle, 0.05);
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(ActuatorIOOutputs outputs) {
    desiredAngle = Radians.of(outputs.positionRads);

    controller.setReference(desiredAngle.in(Rotations), ControlType.kPosition);
  }
}