package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.VortechsUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem responsible for the intake rolling mechanism */
public class Intake extends SubsystemBase {

  // advantage kit logging
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // useful for a flexible hardware interface and for advantage kit logging
  private final IntakeIO moduleIO;

  // these are for advantage kit state logging and/or for keeping track of key variables
  @AutoLogOutput private boolean isOnTarget = false;
  @AutoLogOutput private double targetPosition;
  @AutoLogOutput private double currentPosition;

  @AutoLogOutput private double rollerPower;
  @AutoLogOutput private double currentSpeed;

  /**
   * Constructor for the Intake subsystem.
   *
   * @param moduleIO Hardware interface for Intake motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Intake(IntakeIO moduleIO) {
    this.moduleIO = moduleIO;

    currentSpeed = moduleIO.getSpeed();
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
    currentPosition = moduleIO.getPosition();

    if (targetPosition > IntakeConstants.MAX_POSITION) {
      targetPosition = IntakeConstants.MAX_POSITION;
    }

    if (targetPosition < IntakeConstants.MIN_POSITION) {
      targetPosition = IntakeConstants.MIN_POSITION;
    }

    isOnTarget = isOnTarget();

    // Clamp target speed to prevent exceeding limits
    rollerPower = VortechsUtil.clamp(rollerPower, Constants.IntakeConstants.MAX_TARGET_SPEED);

    moduleIO.setTargetPosition(targetPosition);
    moduleIO.set(rollerPower);
  }

  public void setPosition(double position) {
    targetPosition = position;
  }

  /** sets the roller motors, 0-1 */
  public void setRollers(double voltage) {

    // clamp speed to prevent exceeding limits
    voltage = VortechsUtil.clamp(voltage, Constants.IntakeConstants.MAX_MANUAL_SPEED);

    System.out.println("Above speed limit; rate limiting Intake speed.");
    moduleIO.set(voltage);
  }

  /** Holds the current position using braking mode. */
  public void holdPositionBrake() {
    moduleIO.stop();
  }

  // returns wether or not the elevaotr is on target
  public boolean isOnTarget() {

    double diff = Math.abs(targetPosition - currentPosition);

    return IntakeConstants.POS_TOLERANCE > diff;
  }

  /** resets encoders to read 0 and resets PID (setting it to begin at current height) */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }

  // gets the roller speed
  public double getRollerSpeed() {
    return moduleIO.getSpeed();
  }

  public double getPosition() {
    return currentPosition;
  }

  // commands

  // sets the manual override speed of this command. Uses a regular double
  public Command setManualOverrideCommand(double speed) {
    return new RunCommand(() -> this.setRollers(speed), this);
  }

  // sets the manual override speed of this command. Uses a double supplier
  public Command setManualOverrideCommand(DoubleSupplier speed) {
    return new RunCommand(() -> this.setRollers(speed.getAsDouble()), this);
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
}
