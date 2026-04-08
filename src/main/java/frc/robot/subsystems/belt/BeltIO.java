package frc.robot.subsystems.belt;

import org.littletonrobotics.junction.AutoLog;

public interface BeltIO {
  @AutoLog
  public static class BeltIOInputs {
    double speed;
    boolean isBraked = true;
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
   * sets the voltage of the motor
   *
   * @param voltage
   */
  public default void setVoltage(double voltage) {}

  public default void setBraked(boolean braked) {}
}
