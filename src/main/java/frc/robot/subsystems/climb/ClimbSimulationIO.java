package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbSimulationIO implements ClimbIO {

  private final DCMotorSim leftMotorSim;
  private final DCMotorSim rightMotorSim;
  private Servo servo;
  private double servoPosition = 0.0;

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

    // this the servo
    this.servo = new Servo(1);
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    leftMotorSim.update(0.02);
    rightMotorSim.update(0.02);

    inputs.motorLeftPosition = leftMotorSim.getAngularPositionRotations();
    inputs.motorRightPosition = rightMotorSim.getAngularPositionRotations();
    inputs.motorLeftAmps = leftMotorSim.getCurrentDrawAmps();
    inputs.motorRightAmps = rightMotorSim.getCurrentDrawAmps();

    inputs.servoPosition = servoPosition;
  }

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    leftMotorSim.setInputVoltage(leftSpeed * 12);
    rightMotorSim.setInputVoltage(rightSpeed * 12);
  }

  @Override
  public void setServo(double position) {
    this.servoPosition = position;
    servo.setAngle(position);
  }

  @Override
  public void stop() {
    leftMotorSim.setInputVoltage(0);
    rightMotorSim.setInputVoltage(0);
  }
}
