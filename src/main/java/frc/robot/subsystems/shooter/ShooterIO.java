package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    double speed;
    double targetSpeed;
    double amps;
    double voltage;
    boolean isOnTarget;
    boolean isBraked;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

  /** set's rotations/sec of the motor, uses closed loop PID */
  public default void setSpeed(double speed) {}

  /**
   * @return returns rotations/sec
   */
  public default double getSpeed() {
    return 0;
  }

  public default boolean isOnTarget() {
    return false;
  }

  public default void setVoltage(double voltage) {}

  public default void setBraked(boolean braked) {}
}
