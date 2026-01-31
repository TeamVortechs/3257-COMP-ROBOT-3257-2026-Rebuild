package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double motorLeftPosition = 0.0;
    public double motorRightPosition = 0.0;

    public double motorLeftCurrentSupplyAmps = 0.0;
    public double motorRightCurrentSupplyAmps = 0.0;

    public double motorLeftCurrentStatorAmps = 0;
    public double motorRightCurrentStatorAmps = 0;

    public double motorLeftVolts = 0.0;
    public double motorRightVolts = 0.0;

    public double motorLeftSpeed = 0.0;
    public double motorRightSpeed = 0.0;

    public double servoPosition = 0.0;
  }

  public default void updateInputs(ClimbIOInputsAutoLogged inputs) {}

  /**
   * set speed of each motor... needs to be configured to move by distance instead of speed
   *
   * @param leftSpeed
   * @param rightSpeed
   */
  public default void setSpeeds(double leftSpeed, double rightSpeed) {}

  /**
   * set position of servo
   *
   * @param position
   */
  public default void setServo(double position) {}

  /**
   * set pos of motors
   *
   * @param leftPosition
   * @param rightPosition
   */
  public default void setPositions(double leftPosition, double rightPosition) {}

  /**
   * set voltage of left motor
   *
   * @param leftVoltage
   */
  public default void setLeftVoltage(double leftVoltage) {}

  /**
   * set right voltage
   *
   * @param rightVoltage
   */
  public default void setRightVoltage(double rightVoltage) {}
  /** stop the motors/servos, shutdown scenario? */
  public default void stop() {}
}
