package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class FeederSparkIO implements FeederIO {
  final SparkMax m_motor;
  private double targetSpeed = 0;

  public FeederSparkIO(int id) {
    m_motor = new SparkMax(id, MotorType.kBrushed);
  }

  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    inputs.supplyCurrentAmps = m_motor.getOutputCurrent();
    inputs.voltage = m_motor.getBusVoltage();
    inputs.speed = 0;


    inputs.targetSpeed = targetSpeed;
  }

  /*
   * sets speed -1 to 1
   */
  public void setPercentMotorOutput(double speed) {
    targetSpeed = speed;
    m_motor.set(speed);
  }

  public void stop() {
    targetSpeed = 0;
    m_motor.set(0);
  }

  /*
   * returns speed in RPM
   */
  public double getSpeed() {
    return 0;
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }

  @Override
  public double getTargetSpeed() {
    return targetSpeed;
  }
}
