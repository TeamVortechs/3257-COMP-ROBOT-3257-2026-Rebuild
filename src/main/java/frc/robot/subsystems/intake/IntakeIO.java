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
    double rollerAmps = 0;
    double rollerVolts = 0;
    double rollerSpeed = 0;

    double positionAmps = 0;
    double positionVolts = 0;
    double positionSpeed = 0;

    double position = 0;
    double targetPosition = 0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {}

  // getters for motors

  // gets the height of the arm in meters
  public default double getCurrent() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  // setters for motors
  public default void setRollerVoltage(double volt) {}

  public default void setPositionVoltage(double volt) {

  }

  // sets the position of the arm.
  public default void setTargetPosition(double position) {}

  public default double getTargetPosition() {
    return 0;
  }
  // misc methods

  // rebuilds the pid constants of the motors
  public default void rebuildMotorsPID() {}

  /** Stops the motor immediately */
  public default void stop() {}
  ;

  public default void resetEncoders() {}

  public default void setBraked(boolean braked) {}

  // gets the highest possible height of the arm in radians
  public default double getMaxPosition() {
    return 0;
  }

  // gets the height of the arm in meters
  public default double getPosition() {
    return 0;
  }

  public default boolean isMaxPosition() {
    return false;
  }

  public default double getSpeed() {
    return 0;
  }

  public default boolean checkIfStalled() {
    return false;
  }
}
