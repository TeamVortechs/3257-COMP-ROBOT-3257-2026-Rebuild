package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for arm Module IO implementations. This abstracts all hardware interactions for the
 * arm.
 */

/*
units:
radian
radians
seconds
volts
amperes
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    double rollerAmpsStator = 0;
    double rollerAmpsSupply = 0;
    double rollerVolts = 0;
    double rollerSpeed = 0;

    double positionAmpsStator = 0;
    double positionAmpsSupply = 0;
    double positionVolts = 0;
    double positionSpeed = 0;

    double position = 0;
    double targetPosition = 0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  /**
   * updates inputs for logging purposes
   *
   * @param inputsAutoLogged
   */
  public default void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {}

  // getters for motors

  /**
   * gets stator current going in the roller motor
   *
   * @return gets stator current going in the motor in amps
   */
  public default double getCurrent() {
    return 0;
  }

  /**
   * @return voltage of the roller motor
   */
  public default double getVoltage() {
    return 0;
  }

  /**
   * sets roller voltage to param
   *
   * @param volt
   */
  public default void setRollerVoltage(double volt) {}

  /**
   * set's position voltage to param
   *
   * @param volt
   */
  public default void setPositionVoltage(double volt) {}

  /**
   * set's position of position motor
   *
   * @param position rotations
   */
  public default void setTargetPosition(double position) {}

  /**
   * gets target position of the position motor
   *
   * @return returns target pos of position motor
   */
  public default double getTargetPosition() {
    return 0;
  }
  // misc methods

  /** rebuilds the pid constants of the motors */
  public default void rebuildMotorsPID() {}

  /** Stops the motor immediately */
  public default void stop() {}

  /** reset position encoder */
  public default void resetEncoders() {}

  /**
   * brakes all motors if param is true, otherwise coasts if false
   *
   * @param braked
   */
  public default void setBraked(boolean braked) {}

  /**
   * gets the highest possible height of the arm in radians
   *
   * @return highest poss height of arm in radians
   */
  public default double getMaxPosition() {
    return 0;
  }

  /**
   * get position of the position motor
   *
   * @return
   */
  public default double getPosition() {
    return 0;
  }

  /**
   * checks if position motor is within certain tolerance of the max position
   *
   * @return
   */
  public default boolean isMaxPosition() {
    return false;
  }

  /**
   * @return returns velocity of roller in rpm
   */
  public default double getSpeed() {
    return 0;
  }

  /**
   * returns true if motor is stalled
   *
   * <p>does this by seeing if the requested volts is above a certain amnt
   *
   * @return
   */
  public default boolean checkIfStalled() {
    return false;
  }
}
