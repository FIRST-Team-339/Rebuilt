package us.kilroyrobotics.subsystems.launcher.flywheel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import us.kilroyrobotics.Constants.LauncherConstants.FlywheelConstants;

public class FlywheelIOSparkMax implements FlywheelIO {
  private final SparkMax motor;
  private final SparkMax followerMotor;

  private final BangBangController controller;
  private final SimpleMotorFeedforward feedforward;

  public FlywheelIOSparkMax(int motorId, int followerMotorId) {
    this.motor = new SparkMax(motorId, MotorType.kBrushless);
    this.followerMotor = new SparkMax(followerMotorId, MotorType.kBrushless);

    this.controller = new BangBangController(50);
    this.feedforward = new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV);

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    motorConfig.idleMode(IdleMode.kCoast);
    motorConfig.inverted(true);
    motorConfig.smartCurrentLimit(40);
    motorConfig.closedLoop.minOutput(0);

    SparkMaxConfig followerMotorConfig = new SparkMaxConfig();
    followerMotorConfig.idleMode(IdleMode.kCoast);
    followerMotorConfig.smartCurrentLimit(40);
    followerMotorConfig.follow(motor, true);
    followerMotorConfig.closedLoop.minOutput(0);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(
        followerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.connected = !Double.isNaN(motor.getAppliedOutput());
    inputs.positionRads = Rotations.of(motor.getEncoder().getPosition()).in(Radians);
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.appliedVoltage = motor.getAppliedOutput();
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();

    inputs.followerConnected = !Double.isNaN(followerMotor.getAppliedOutput());
    inputs.followerSupplyCurrentAmps = followerMotor.getOutputCurrent();
    inputs.followerTempCelsius = followerMotor.getMotorTemperature();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    // motor.setVoltage(
    //     controller.calculate(motor.getEncoder().getVelocity(), outputs.velocityRPM) * 12.0
    //         + 0.9 * feedforward.calculate(outputs.velocityRPM));
    // motor.set(controller.calculate(motor.getEncoder().getVelocity(), outputs.velocityRPM));
    motor.setVoltage(feedforward.calculate(outputs.velocityRPM));
  }
}
