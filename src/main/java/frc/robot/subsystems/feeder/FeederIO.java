package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    double speed;
    double targetSpeed;
    double supplyCurrentAmps;
    double statorCurrentAmps;
    double voltage;
    boolean isOnTargetSpeed;
    double temperatureCelsius;

    boolean isBraked = true;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(FeederIOInputsAutoLogged inputs) {}
  /**
   * @param speed sets speed in RPM
   */
  public default void setPercentMotorOutput(double speed) {}
  /**
   * @return returns speed in RPM
   */
  public default double getSpeed() {
    return 0;
  }

  /**
   * @return wether or not the speed and target speed are within tolerance
   */
  public default boolean isOnTargetSpeed() {
    return false;
  }

  /**
   * sets the voltage of the motor
   *
   * @param voltage
   */
  public default void setVoltage(double voltage) {}

  public default void setBraked(boolean braked) {}
}
