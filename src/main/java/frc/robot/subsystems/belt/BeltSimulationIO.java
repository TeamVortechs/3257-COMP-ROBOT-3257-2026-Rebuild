package frc.robot.subsystems.belt;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class BeltSimulationIO implements BeltIO {
  // the motor that spins the things to shoot balls?
  private final DCMotorSim rollerMotorsSim;
  private double targetSpeed = 0;

  public BeltSimulationIO() {
    // now this motor exists in advantage kit?
    this.rollerMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
  }

  // update inputs on roller motors
  @Override
  public void updateInputs(BeltIOInputsAutoLogged inputs) {
    inputs.supplyCurrentAmps = rollerMotorsSim.getCurrentDrawAmps();
    inputs.voltage = rollerMotorsSim.getInputVoltage();
    inputs.speed = rollerMotorsSim.getAngularVelocityRPM();

    inputs.targetOutput = targetSpeed;

    rollerMotorsSim.update(0.02);
  }

  @Override
  public void setPercentMotorOutput(double speed) {
    setVoltage(speed / 502.747);
    targetSpeed = speed;
  }

  @Override
  public void setVoltage(double voltage) {
    rollerMotorsSim.setInputVoltage(voltage);
  }

  public void stop() {
    targetSpeed = 0;
    setVoltage(0);
  }

  public double getSpeed() {
    return rollerMotorsSim.getAngularVelocityRPM();
  }
}
