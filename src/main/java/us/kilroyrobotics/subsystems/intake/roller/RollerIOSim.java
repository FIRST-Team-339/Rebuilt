package us.kilroyrobotics.subsystems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private final DCMotor gearbox = DCMotor.getNeoVortex(1);
  private final DCMotorSim simMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.0001, 1), gearbox);
  private double appliedVoltage = 0.0;

  public RollerIOSim() {}

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    simMotor.update(0.02);

    inputs.connected = true;
    inputs.positionRads = simMotor.getAngularPositionRad();
    inputs.velocityRadsPerSec = simMotor.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = simMotor.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        gearbox.getCurrent(simMotor.getAngularVelocityRadPerSec(), appliedVoltage);
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(RollerIOOutputs outputs) {
    if (DriverStation.isDisabled()) {
      appliedVoltage = 0.0;
    } else {
      appliedVoltage = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
    }
    simMotor.setInputVoltage(appliedVoltage);
  }
}
