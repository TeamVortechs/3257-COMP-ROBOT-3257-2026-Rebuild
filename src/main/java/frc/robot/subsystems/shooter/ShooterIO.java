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
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }

  public default boolean isOnTarget() {
    return false;
  }

  public default void setVoltage(double voltage) {}
}
