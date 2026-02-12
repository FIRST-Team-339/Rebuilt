package us.kilroyrobotics.subsystems.intake.actuator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import us.kilroyrobotics.Constants.IntakeConstants.ActuatorConstants;

public class ActuatorIOSim implements ActuatorIO {
  private final DCMotor gearbox = DCMotor.getNEO(1);
  private final DCMotorSim simMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(gearbox, 0.025, ActuatorConstants.kGearing), gearbox);

  private PIDController controller =
      new PIDController(ActuatorConstants.kP, ActuatorConstants.kI, ActuatorConstants.kD, 0.02);
  private double appliedVoltage = 0.0;
  private double currentOutput = 0.0;
  private double prevOutput = 0.0;

  /** Creates a new AcuatorIOSim. */
  public ActuatorIOSim() {}

  @Override
  public void updateInputs(ActuatorIOInputs inputs) {
    appliedVoltage = gearbox.getVoltage(currentOutput, simMotor.getAngularVelocityRadPerSec());

    simMotor.update(0.02);
    simMotor.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));

    inputs.connected = true;
    inputs.positionRads = simMotor.getAngularPositionRad() / ActuatorConstants.kGearing;
    inputs.atSetpoint = controller.atSetpoint();
    inputs.velocityRadsPerSec = simMotor.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = simMotor.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        gearbox.getCurrent(simMotor.getAngularVelocityRadPerSec(), appliedVoltage);
    inputs.tempCelsius = 0.0;

    prevOutput = simMotor.getAngularPositionRad() / ActuatorConstants.kGearing;
  }

  @Override
  public void applyOutputs(ActuatorIOOutputs outputs) {
    if (DriverStation.isDisabled()) {
      currentOutput = controller.calculate(prevOutput, 0.0);
      simMotor.setInputVoltage(0.0);
    } else {
      currentOutput = controller.calculate(prevOutput, outputs.positionRads);
    }
  }
}
