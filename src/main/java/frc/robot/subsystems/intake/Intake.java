package frc.robot.subsystems.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.VortechsUtil;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

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
    moduleIO.setRollerVoltage(rollerPower);
  }

  public void setPosition(double position) {
    targetPosition = position;
  }

  /** sets the roller motors, 0-1 */
  public void setRollers(double voltage) {

    // clamp speed to prevent exceeding limits
    voltage = VortechsUtil.clamp(voltage, Constants.IntakeConstants.MAX_MANUAL_SPEED);

    moduleIO.setRollerVoltage(voltage);
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
  public Command setRollerCommand(double speed) {
    return new RunCommand(() -> this.setRollers(speed), this);
  }

  // sets the manual override speed of this command. Uses a double supplier
  public Command setRollerCommand(DoubleSupplier speed) {
    return new RunCommand(() -> this.setRollers(speed.getAsDouble()), this);
  }

  public Command setPositionCommand(double position) {
    return new RunCommand(() -> this.setPosition(position), this);
  }

  public Command setSpeedAndPositionCommand(double position, double speed) {
    return Commands.parallel(
        new RunCommand(() -> this.setPosition(position)),
        new RunCommand(() -> this.setRollers(speed), this));
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
        new SysIdRoutine.Mechanism((volts) -> moduleIO.setRollerVoltage(volts.in(Volts)), null, this));
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
            (volts) -> moduleIO.setPositionVoltage(volts.in(Volts)), null, this));
  }

}
