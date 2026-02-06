package us.kilroyrobotics.subsystems.launcher.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor gearbox = DCMotor.getNeoVortex(1);
  private final DCMotorSim simMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.025, 1), gearbox);

  private PIDController controller = new PIDController(0.5, 0, 0, 0.02);
  private double appliedVoltage = 0.0;
  private double currentOutput = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    appliedVoltage = gearbox.getVoltage(currentOutput, simMotor.getAngularVelocityRadPerSec());

    simMotor.update(0.02);

    inputs.connected = true;
    inputs.positionRads = simMotor.getAngularPositionRad();
    inputs.velocityRPM = simMotor.getAngularVelocityRPM();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = simMotor.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        gearbox.getCurrent(simMotor.getAngularVelocityRadPerSec(), appliedVoltage);
    inputs.tempCelsius = 0.0;

    inputs.followerConnected = true;
    inputs.followerSupplyCurrentAmps = simMotor.getCurrentDrawAmps();
    inputs.followerTempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    currentOutput = controller.calculate(outputs.velocityRPM);
  }
}
