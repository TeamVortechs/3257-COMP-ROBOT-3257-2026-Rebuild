package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbSimulationIO implements ClimbIO {

  private final DCMotorSim leftMotorSim;
  private final DCMotorSim rightMotorSim;

  private double servoTargetPosition = 0.0;

  private PIDController positionPIDController = new PIDController(1, 0, 0);

  private boolean manual = true;

  private double targetLeftPos = 0;
  private double targetRightPos = 0;

  public ClimbSimulationIO() {
    // left and right, just like the real thing
    this.leftMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));

    this.rightMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    leftMotorSim.update(0.02);
    rightMotorSim.update(0.02);

    inputs.motorLeftPosition = leftMotorSim.getAngularPositionRotations();

    inputs.motorLeftCurrentSupplyAmps = leftMotorSim.getCurrentDrawAmps();

    inputs.motorLeftVolts = leftMotorSim.getInputVoltage();

    inputs.motorLeftSpeed = leftMotorSim.getAngularVelocityRPM();

    inputs.servoTargetPosition = servoTargetPosition;

    if (!manual) {
      leftMotorSim.setInputVoltage(
          positionPIDController.calculate(
              leftMotorSim.getAngularPositionRotations(), targetLeftPos));
      rightMotorSim.setInputVoltage(
          positionPIDController.calculate(
              rightMotorSim.getAngularPositionRotations(), targetRightPos));
    }
  }

  @Override
  public void setVoltage(double leftSpeed) {
    leftMotorSim.setInputVoltage(leftSpeed / 502.747);

    manual = true;
  }
}
