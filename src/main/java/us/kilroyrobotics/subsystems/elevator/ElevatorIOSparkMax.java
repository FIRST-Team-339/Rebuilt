package us.kilroyrobotics.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import us.kilroyrobotics.Constants.ElevatorConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leaderMotor;
  private final SparkMax followerMotor;

  private final SparkClosedLoopController controller;

  private Distance desiredPosition = Meters.of(0.0);

  /**
   * Creates a new ElevatorIOSparkMax.
   *
   * @param leaderMotorId the CAN ID of the leader motor
   * @param followerMotorId the CAN ID of the follower motor
   */
  public ElevatorIOSparkMax(int leaderMotorId, int followerMotorId) {
    this.leaderMotor = new SparkMax(leaderMotorId, MotorType.kBrushless);
    this.followerMotor = new SparkMax(followerMotorId, MotorType.kBrushless);
    this.controller = leaderMotor.getClosedLoopController();

    // Configure leader motor
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP.get(), ElevatorConstants.kI.get(), ElevatorConstants.kD.get());
    leaderConfig.idleMode(IdleMode.kBrake);
    leaderConfig.smartCurrentLimit(40);
    leaderConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure follower motor to follow leader
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(leaderMotorId, false); // false = same direction
    followerConfig.idleMode(IdleMode.kBrake);
    followerConfig.smartCurrentLimit(40);

    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Reset encoder position to 0
    leaderMotor.getEncoder().setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected = !leaderMotor.hasStickyFault();
    inputs.positionMeters = leaderMotor.getEncoder().getPosition();
    inputs.velocityMetersPerSec = leaderMotor.getEncoder().getVelocity();
    inputs.atSetpoint = Meters.of(inputs.positionMeters).isNear(desiredPosition, 0.02);
    inputs.appliedVoltage = leaderMotor.getAppliedOutput() * leaderMotor.getBusVoltage();
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = leaderMotor.getOutputCurrent() + followerMotor.getOutputCurrent();
    inputs.tempCelsius = Math.max(leaderMotor.getMotorTemperature(), followerMotor.getMotorTemperature());
    
    // Update PID coefficients if they've changed (tunable)
    controller.setP(ElevatorConstants.kP.get());
    controller.setI(ElevatorConstants.kI.get());
    controller.setD(ElevatorConstants.kD.get());
  }

  @Override
  public void applyOutputs(ElevatorIOOutputs outputs) {
    desiredPosition = Meters.of(outputs.positionMeters);

    controller.setSetpoint(outputs.positionMeters, ControlType.kPosition);
  }
}
