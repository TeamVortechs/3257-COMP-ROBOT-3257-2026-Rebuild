package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CIntake;
import frc.robot.subsystems.canrange.RangeFinder;
import frc.robot.util.FieldMovement.VortechsUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem responsible for the intake rolling mechanism */
public class Intake extends SubsystemBase {

  // advantage kit logging
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // useful for a flexible hardware interface and for advantage kit logging
  private final IntakeIO moduleIO;

  private final RangeFinder canRange;

  // these are for advantage kit state logging and/or for keeping track of key variables
  @AutoLogOutput private double currentSpeed = 0.0;
  @AutoLogOutput private double targetSpeed = 0.0;
  @AutoLogOutput private boolean isOnTarget = false;
  @AutoLogOutput private boolean manualOverride = false;

  /**
   * Constructor for the Intake subsystem.
   *
   * @param moduleIO Hardware interface for Intake motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Intake(IntakeIO moduleIO, RangeFinder canRange) {
    this.moduleIO = moduleIO;
    this.canRange = canRange;

    currentSpeed = moduleIO.getSpeed();
    targetSpeed = moduleIO.getTargetSpeed();
    setTargetSpeed(currentSpeed);
  }

  @Override
  public void periodic() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // check to see if the module is stalling; if so, then stop the motors and cancel the next
    // movement

    if (moduleIO.checkIfStalled()) {
      System.out.println("Intake HAS STALLED ");
      moduleIO.stop();
      return;
    }

    currentSpeed = moduleIO.getSpeed();

    // if manual override return
    if (manualOverride) {
      return;
    }

    isOnTarget = isOnTarget();

    // Clamp target speed to prevent exceeding limits
    targetSpeed = VortechsUtil.clamp(targetSpeed, Constants.CIntake.MAX_TARGET_SPEED);

    moduleIO.setTargetSpeed(targetSpeed);
  }

  /** Sets a new target height for the Intake using PID control. */
  public void setTargetSpeed(double targetSpeed) {
    manualOverride = false;

    double clampedSpeed = VortechsUtil.clamp(targetSpeed, Constants.CIntake.MAX_TARGET_SPEED);

    // Clamp target speed to prevent exceeding limits
    this.targetSpeed = clampedSpeed;
  }

  /** Allows manual control of the Intake, bypassing PID. */
  public void setManualVoltage(double voltage) {
    manualOverride = true;

    // clamp speed to prevent exceeding limits
    voltage = VortechsUtil.clamp(voltage, Constants.CIntake.MAX_MANUAL_SPEED);

    System.out.println("Above speed limit; rate limiting Intake speed.");
    moduleIO.setVoltage(voltage);
  }

  /** Holds the current position using PID control. */
  public void holdPositionPID() {
    System.out.println("holding position pid");
    manualOverride = false;
    if (Math.abs(targetSpeed - currentSpeed) > CIntake.SPEED_TOLERANCE) {

      targetSpeed = currentSpeed;
      moduleIO.setTargetPosition(moduleIO.getPosition());
    }
  }

  /** Holds the current position using braking mode. */
  public void holdPositionBrake() {
    manualOverride = true;
    moduleIO.stop();
  }

  /** Emergency stop function that immediately disables motor output. */
  public void emergencyStop() {
    moduleIO.stop();
    manualOverride = true;
  }

  // returns wether or not the Intake is close to the floor
  public boolean isOnFloor() {
    return getCurrentAngle() < 0.1;
  }

  // gest the current height of the Intake motor
  public double getCurrentAngle() {
    return currentSpeed;
  }

  public double getTargetSpeed() {
    return targetSpeed;
  }

  // returns wether or not the elevaotr is on target
  public boolean isOnTarget() {
    return (Math.abs(currentSpeed - targetSpeed) < CIntake.SPEED_TOLERANCE);
  }

  /** resets encoders to read 0 and resets PID (setting it to begin at current height) */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }

  // gets the roller speed
  public double getRollerSpeed() {
    return moduleIO.getSpeed();
  }

  // resets the motors pid
  public void rebuildMotorsPID() {
    moduleIO.rebuildMotorsPID();
  }

  // commands

  // sets the target height of the subsystem. Ends immediately
  public Command instantSetTargetSpeedCommand(double targetSpeed) {
    return new InstantCommand(() -> this.setTargetSpeed(targetSpeed), this);
  }

  public Command setTargetSpeedCommand(double targetSpeed) {
    return this.run(() -> this.setTargetSpeed(targetSpeed));
  }

  // sets the target height of the subsystem. Ends when the subsystem reaches this height
  public Command setTargetSpeedCommandConsistentEnd(double targetSpeed) {
    return new InstantCommand(() -> this.setTargetSpeed(targetSpeed), this)
        .andThen(new WaitUntilCommand(() -> this.isOnTarget));
  }

  // sets the manual override speed of this command. Uses a regular double
  public Command setManualOverrideCommand(double speed) {
    return new RunCommand(() -> this.setManualVoltage(speed), this);
  }

  // sets the manual override speed of this command. Uses a double supplier
  public Command setManualOverrideCommand(DoubleSupplier speed) {
    return new RunCommand(() -> this.setManualVoltage(speed.getAsDouble()), this);
  }

  // makes the PID hold position. If true it'll use brake if false it'll pid.
  public Command holdSpeedCommand(boolean brake) {
    if (brake) return new InstantCommand(() -> this.holdPositionBrake(), this);

    return new InstantCommand(() -> this.holdPositionPID(), this);
  }

  // resets the encoders of the wrist
  public Command resetEncodersCommand() {
    return new InstantCommand(() -> this.resetEncoders());
  }

  // intakes until the canrange finds distance less than the given distance
  // public Command intakeUntilCanRangeIsDetected(double speed, double distance) {
  //   return new RunCommand(() -> this.setTargetSpeed(speed), this)
  //       .until(() -> canRange.getCanDistance() < distance);
  // }

  // simple command that requires this subsystem
  public Command requireSubsystemCommand() {
    return new InstantCommand(() -> {}, this);
  }

  // rebuilds the motor pid
  public Command rebuildMotorsPIDCommand() {
    return new InstantCommand(() -> this.rebuildMotorsPID());
  }
}