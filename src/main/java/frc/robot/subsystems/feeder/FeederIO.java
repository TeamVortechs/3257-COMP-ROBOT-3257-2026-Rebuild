package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    double speed;
    double targetSpeed;
    double amps;
    double voltage;
    boolean isOnTarget;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(FeederIOInputsAutoLogged inputs) {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }

  public default boolean isOnTarget() {
    return false;
  }

  public default void setVoltage(double voltage) {}
}
