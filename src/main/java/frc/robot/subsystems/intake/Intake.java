package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem responsible for the intake rolling mechanism */
public class Intake extends SubsystemBase {

  // advantage kit logging
  private IntakeIOInputsAutoLogged inputs;

  // useful for a flexible hardware interface and for advantage kit logging
  private final IntakeIO intakeIO;

  private final Notifier logger;
  /**
   * Constructor for the Intake subsystem.
   *
   * @param intakeIO Hardware interface for Intake motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    this.inputs = new IntakeIOInputsAutoLogged();

    logger =
        new Notifier(
            () -> {
              intakeIO.updateInputs(inputs);
              Logger.processInputs("intake", inputs);
            });

    logger.startPeriodic(1 / IntakeConstants.FREQUENCY_HZ);
  }

  @Override
  public void periodic() {
    // check to see if the module is stalling; if so, then stop the motors and cancel the next
    // movement

    if (intakeIO.checkIfStalled()) {
      System.out.println("Intake HAS STALLED ");
      intakeIO.stop();
      return;
    }

    // Clamp target speed to prevent exceeding limits

  }

  public void setPosition(double targetPosition) {

    if (targetPosition > IntakeConstants.MAX_POSITION) {
      targetPosition = IntakeConstants.MAX_POSITION;
    }

    if (targetPosition < IntakeConstants.MIN_POSITION) {
      targetPosition = IntakeConstants.MIN_POSITION;
    }

    intakeIO.setPositionControl(targetPosition);
  }

  /** sets the roller motors, -1-1 */
  public void setRollersVoltage(double voltage) {

    intakeIO.setRollerVoltage(voltage);
  }

  /** Holds the current position using braking mode. */
  public void stop() {
    intakeIO.stop();
  }

  public boolean isOnTarget() {

    double diff = Math.abs(intakeIO.getTargetPosition() - intakeIO.getPosition());

    return IntakeConstants.POS_TOLERANCE > diff;
  }

  /** resets encoders to read 0 and resets PID (setting it to begin at current height) */
  public void resetEncoders() {
    intakeIO.resetEncoders();
  }

  // gets the roller speed
  public double getRollerSpeed() {
    return intakeIO.getSpeed();
  }

  public double getPosition() {
    return intakeIO.getPosition();
  }

  // commands

  // sets the manual override speed of this command. Uses a regular double
  public Command setRollerVoltageCommand(double speed) {
    return new RunCommand(() -> this.setRollersVoltage(speed), this);
  }

  // sets the manual override speed of this command. Uses a double supplier
  public Command setRollerVoltageCommand(DoubleSupplier speed) {
    return new RunCommand(() -> this.setRollersVoltage(speed.getAsDouble()), this);
  }

  public Command setPositionCommand(double position) {
    return new RunCommand(() -> this.setPosition(position), this);
  }

  public Command setRollerVoltageAndPositionCommand(double position, double voltage) {
    return Commands.parallel(
        new RunCommand(() -> this.setPosition(position)),
        new RunCommand(() -> this.setRollersVoltage(voltage), this));
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

  // the constants here should probably be more and move but that's later when this is transferred
  // to the right project
  // add this to the robot class or this won't work: SignalLogger.setPath("/media/sda1/");
  /**
   * Gets the system identification routine for this specific subsystem
   *
   * @return the sysid routine
   */
  /** Build SysId Routine for the Left Motor */
  public SysIdRoutine builtRollerSysIDRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(IntakeConstants.RAMP_RATE_VOLTS_ROLLER_SYSID).per(Units.Second),
            Volts.of(IntakeConstants.DYNAMIC_STEP_VOLTS_ROLLER_SYSID),
            null,
            (state) -> SignalLogger.writeString("intakeRollers", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> intakeIO.setRollerVoltage(volts.in(Volts)), null, this));
  }

  /** Build SysId Routine for the Right Motor */
  public SysIdRoutine buildPositionSysIDRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(IntakeConstants.RAMP_RATE_VOLTS_POSITION_SYSID).per(Units.Second),
            Volts.of(IntakeConstants.DYNAMIC_STEP_VOLTS_POSITION_SYSID),
            null,
            (state) -> SignalLogger.writeString("intakePostion", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> intakeIO.setPositionVoltage(volts.in(Volts)), null, this));
  }
}
