package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;

public class ShooterSparkIO implements ShooterIO {
  final SparkMax m_motor = new SparkMax(Constants.ShooterConstants.ID, MotorType.kBrushed);
  private RelativeEncoder m_encoder = m_motor.getEncoder();
  private double targetSpeed = 0;
  private boolean isOnTarget = false;

  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    inputs.amps = m_motor.getOutputCurrent();
    inputs.voltage = m_motor.getBusVoltage();
    inputs.speed = m_encoder.getVelocity(); // in RPM

    inputs.targetSpeed = targetSpeed;
    inputs.isOnTarget = isOnTarget;
  }

  /*
   * sets speed -1 to 1
   */
  public void setSpeed(double speed) {
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
    return m_encoder.getVelocity();
  }

  public boolean isOnTarget() {
    return Math.abs((getSpeed() - targetSpeed)) <= Constants.ShooterConstants.TOLERANCE;
  }

  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
