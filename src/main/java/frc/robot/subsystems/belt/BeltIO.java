package frc.robot.subsystems.belt;

import org.littletonrobotics.junction.AutoLog;

public interface BeltIO {
  @AutoLog
  public static class BeltIOInputs {
    double speed;
    double targetOutput;
    double supplyCurrentAmps;
    double statorCurrentAmps;
    double voltage;
    boolean isBraked;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(BeltIOInputsAutoLogged inputs) {}

  /**
   * sets the speed as percentage of the battery voltage. From -1 to 1.
   *
   * @param speed
   */
  public default void setPercentMotorOutput(double speed) {}

  /**
   * @return the speed in rotations per second of the motor
   */
  public default double getSpeed() {
    return 0;
  }

  /**
   * sets the voltage of the motor
   *
   * @param voltage
   */
  public default void setVoltage(double voltage) {}

  public default void setBraked(boolean braked) {}
}
