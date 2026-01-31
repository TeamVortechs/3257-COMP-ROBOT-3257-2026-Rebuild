package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    double speed;
    double targetSpeed;
    double amps;
    double voltage;
    boolean isOnTargetSpeed;
    boolean isBraked;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(FeederIOInputsAutoLogged inputs) {}
  /**
   * @param speed sets speed in RPM
   */
  public default void setSpeed(double speed) {}
  /**
   * @return returns speed in RPM
   */
  public default double getSpeed() {
    return 0;
  }

  public default boolean isOnTargetSpeed() {
    return false;
  }

  public default void setVoltage(double voltage) {}

  public default void setBraked(boolean braked) {}
}
