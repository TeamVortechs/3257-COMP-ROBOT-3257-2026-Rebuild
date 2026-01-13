package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * arm subsystem responsible for controlling the lifting mechanism. Uses PID control for precise
 * movement and prevents unsafe operation via limit switches and software constraints.
 */
public class Arm extends SubsystemBase {

  // advantage kit logging
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // useful for a flexible hardware interface and for advantage kit logging
  private final ArmIO moduleIO;

  // these are for advantage kit state logging and/or for keeping track of key variables
  @AutoLogOutput private double currentHeight = 0.0;
  @AutoLogOutput private double targetHeight = 0.0;
  @AutoLogOutput private boolean isOnTarget = false;
  @AutoLogOutput private boolean manualOverride = false;

  /**
   * Constructor for the Arm subsystem.
   *
   * @param moduleIO Hardware interface for Arm motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Arm(ArmIO moduleIO) {
    this.moduleIO = moduleIO;

    currentHeight = moduleIO.getAngle();
    targetHeight = moduleIO.getAngle();
    setTargetHeight(currentHeight);
  }

  @Override
  public void periodic() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // check to see if the module is stalling; if so, then stop the motors and cancel the next
    // movement

    if (moduleIO.checkIfStalled()) {
      System.out.println("ARM HAS STALLED ");
      moduleIO.stop();
      return;
    }

    currentHeight = moduleIO.getAngle();

    // if manual override do a quick bounds check then retuorn
    if (manualOverride) {

      if (getCurrentAngle() < Constants.ARM_MIN_ANGLE - Constants.ARM_TOLERANCE
          || getCurrentAngle() > Constants.ARM_MAX_ANGLE) {
        System.out.println("ARM OUT OF BOUDNS");
        setManualSpeed(0);
      }
      return;
    }

    isOnTarget = isOnTarget();

    // Clamp target height to prevent exceeding limits
    targetHeight =
        Math.max(Constants.ARM_MIN_ANGLE, Math.min(targetHeight, Constants.ARM_MAX_ANGLE));

    // should be commented bakc in
    moduleIO.PIDVoltage(targetHeight);
  }

  /** Sets a new target height for the arm using PID control. */
  public void setTargetHeight(double height) {

    manualOverride = false;

    targetHeight = Math.max(0.0, Math.min(height, Constants.ARM_MAX_ANGLE));
  }

  /** Allows manual control of the arm, bypassing PID. */
  public void setManualSpeed(double speed) {
    manualOverride = true;

    if (Math.abs(speed) > Constants.ARM_SPEED_LIMIT)
      speed = Math.copySign(Constants.ARM_SPEED_LIMIT, speed);
    System.out.println("Above speed limit; rate limiting ARM speed.");
    moduleIO.setRollerSpeed(speed);
  }

  /** Holds the current position using PID control. */
  public void holdPositionPID() {
    manualOverride = false;
    if (Math.abs(targetHeight - currentHeight) > Constants.ARM_TOLERANCE) {
      targetHeight = currentHeight;
      moduleIO.PIDVoltage(targetHeight);
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

  // returns wether or not the arm is close to the floor
  public boolean isOnFloor() {
    return getCurrentAngle() < 0.1;
  }

  // gest the current height of the arm motor
  public double getCurrentAngle() {
    return currentHeight;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  // returns wether or not the elevaotr is on target
  public boolean isOnTarget() {
    return (Math.abs(currentHeight - targetHeight) < Constants.ARM_TOLERANCE);
  }

  /** resets encoders to read 0 and resets PID (setting it to begin at current height) */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }

  // resets the motors pid
  public void rebuildMotorsPID() {
    moduleIO.rebuildMotorsPID();
  }

  // returns wether or not the arm is clear from the arm
  public boolean isClearFromElevator() {
    return getCurrentAngle() > 0;
  }

  // commands

  // sets the target height of the subsystem. Ends immediately
  public Command setTargetHeightCommand(double targetHeight) {
    return new InstantCommand(() -> this.setTargetHeight(targetHeight), this);
  }

  // sets the target height of the subsystem. Ends when the subsystem reaches this height
  public Command setTargetHeightCommandConsistentEnd(double targetHeight) {
    return new InstantCommand(() -> this.setTargetHeight(targetHeight), this)
        .andThen(new WaitUntilCommand(() -> this.isOnTarget));
  }

  // sets the manual override speed of this command. Uses a regular double
  public Command setManualOverrideCommand(double speed) {
    return new RunCommand(() -> this.setManualSpeed(speed), this);
  }

  // sets the manual override speed of this command. Uses a double supplier
  public Command setManualOverrideCommand(DoubleSupplier speed) {
    return new RunCommand(() -> this.setManualSpeed(speed.getAsDouble()), this);
  }

  // makes the PID hold position. If true it'll use brake if false it'll pid.
  public Command holdPositionCommand(boolean brake) {
    if (brake) return new InstantCommand(() -> this.holdPositionBrake(), this);

    return new InstantCommand(() -> this.holdPositionPID(), this);
  }

  // resets the encoders of the wrist
  public Command resetEncodersCommand() {
    return new InstantCommand(() -> this.resetEncoders());
  }

  // simple command that requires this subsystem
  public Command requireSubsystemCommand() {
    return new InstantCommand(() -> {}, this);
  }

  // rebuilds the motor pid
  public Command rebuildMotorsPIDCommand() {
    return new InstantCommand(() -> this.rebuildMotorsPID());
  }
}
