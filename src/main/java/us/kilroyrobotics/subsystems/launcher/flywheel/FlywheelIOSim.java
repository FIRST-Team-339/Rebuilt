package us.kilroyrobotics.subsystems.launcher.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import us.kilroyrobotics.Constants.LauncherConstants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor gearbox = DCMotor.getNeoVortex(2);
  private final DCMotorSim simMotor =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.025, 1), gearbox);

  private PIDController controller =
      new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD, 0.02);
  private double appliedVoltage = 0.0;
  private double currentOutput = 0.0;
  private double currentRPM = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    appliedVoltage = gearbox.getVoltage(currentOutput, simMotor.getAngularVelocityRadPerSec());

    simMotor.update(0.02);
    simMotor.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));

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

    currentRPM = simMotor.getAngularVelocityRPM();
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (DriverStation.isDisabled()) {
      currentOutput = controller.calculate(currentRPM, 0.0);
    } else {
      currentOutput = controller.calculate(currentRPM, outputs.velocityRPM);
    }
  }
}
