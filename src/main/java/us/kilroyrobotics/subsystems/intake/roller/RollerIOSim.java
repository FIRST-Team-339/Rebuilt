package us.kilroyrobotics.subsystems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private final DCMotor gearbox = DCMotor.getNeoVortex(1);
  private final DCMotorSim simMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.0001, 1), gearbox);

  private PIDController controller = new PIDController(0.1, 0, 0, 0.02);
  private double appliedVoltage = 0.0;
  private double currentOutput = 0.0;

  /** Creates a new RollerIOSim. */
  public RollerIOSim() {}

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    appliedVoltage = gearbox.getVoltage(currentOutput, simMotor.getAngularVelocityRadPerSec());

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
      currentOutput = controller.calculate(simMotor.getAngularVelocityRPM() / 6784, 0.0);
    } else {
      currentOutput =
          controller.calculate(simMotor.getAngularVelocityRPM() / 6784, outputs.appliedOutput);
    }

    simMotor.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
  }
}
