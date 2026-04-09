package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    double speed;

    boolean isBraked = true;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(FeederIOInputsAutoLogged inputs) {}
  /**
   * @param percentage sets speed in volt percent
   */
  public default void setPercentMotorOutput(double percentage) {}
  /**
   * @return returns speed in RPM
   */
  public default double getSpeed() {
    return 0;
  }

  /**
   * @param speed Sets speed in RPS
   */
  public default void setSpeed(double speed) {}
  /**
   * sets the voltage of the motor
   *
   * @param voltage
   */
  public default void setVoltage(double voltage) {}

  public default void setBraked(boolean braked) {}

  public default double getTargetSpeed() {
    return 0;
  }
}
