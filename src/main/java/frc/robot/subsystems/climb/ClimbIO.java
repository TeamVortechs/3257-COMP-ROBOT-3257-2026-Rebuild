package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double motorLeftPosition = 0.0;

    public double motorLeftCurrentSupplyAmps = 0.0;

    public double motorLeftCurrentStatorAmps = 0;

    public double motorLeftVolts = 0.0;

    public double motorLeftSpeed = 0.0;

    public boolean isBraked = true;

    public double motorLeftTemperatureCelsius = 0.0;

    public double servoTargetPosition = 0.0;
  }

  public default void updateInputs(ClimbIOInputsAutoLogged inputs) {}

  /**
   * set speed of each motor... needs to be configured to move by distance instead of speed
   *
   * @param voltage
   */
  public default void setVoltage(double voltage) {}

  /**
   * set position of servo
   *
   * @param position
   */
  public default void setServo(double position) {}

  public default void setBraked(boolean braked) {}
}
