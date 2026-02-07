package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  @AutoLogOutput private boolean isLocked = true;

  @AutoLogOutput private boolean isManual = false;

  @AutoLogOutput private double manualLeftSpeed = 0;
  @AutoLogOutput private double manualRightSpeed = 0;

  @AutoLogOutput private double automaticLeftSetpoint = 0;
  @AutoLogOutput private double automaticRightSetpoint = 0;

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    // climbIO.updateInputs(inputs);
    // Logger.processInputs("Climb", inputs);

    if (isLocked) {
      climbIO.stop();

      setManualSpeeds(0, 0);
      return;
    }

    if (automaticLeftSetpoint > ClimbConstants.MAX_POSITION_LEFT) {
      automaticLeftSetpoint = ClimbConstants.MAX_POSITION_LEFT;
    }

    if (automaticRightSetpoint > ClimbConstants.MAX_POSITION_RIGHT) {
      automaticRightSetpoint = ClimbConstants.MAX_POSITION_RIGHT;
    }

    if (automaticLeftSetpoint < ClimbConstants.MIN_POSITION_LEFT) {
      automaticLeftSetpoint = ClimbConstants.MIN_POSITION_LEFT;
    }

    if (automaticRightSetpoint < ClimbConstants.MIN_POSITION_RIGHT) {
      automaticRightSetpoint = ClimbConstants.MIN_POSITION_RIGHT;
    }

    if (isManual) {
      climbIO.setSpeeds(manualLeftSpeed, manualRightSpeed);
    } else {
      climbIO.setPositions(automaticLeftSetpoint, automaticRightSetpoint);
    }
  }

  /** set servo in degrees */
  public void setServo(double position) {
    climbIO.setServo(position);
  }

  /**
   * Manual control for both motors. Useful if the robot is tilting and you need to adjust one side.
   * sets it to manual
   */
  public void setManualSpeeds(double left, double right) {
    // if (isLocked) {
    //   climbIO.stop();
    //   return;
    // }

    isManual = true;

    manualLeftSpeed = left;
    manualRightSpeed = right;
  }

  /**
   * sets the climber to automatic and sets the positions
   *
   * @param leftPosition
   * @param rightPosition
   */
  public void setPositions(double leftPosition, double rightPosition) {
    isManual = false;
    this.automaticLeftSetpoint = leftPosition;
    this.automaticRightSetpoint = rightPosition;
  }

  public void setLocked(boolean locked) {
    isLocked = locked;
  }

  public boolean isLocked() {
    return isLocked;
  }

  /** makes the climber manual and stops the setpoint */
  public void stop() {
    climbIO.stop();
  }

  public Command setPositionsRunCommand(double leftPosition, double rightPosition) {
    return new RunCommand(() -> setPositions(leftPosition, rightPosition), this);
  }

  public Command setSpeedsRunCommand(double leftPosition, double rightPosition) {
    return new RunCommand(() -> setManualSpeeds(leftPosition, rightPosition), this);
  }

  public Command setIsLockedCommand(BooleanSupplier isLocked) {
    return new InstantCommand(() -> setLocked(isLocked.getAsBoolean()), this);
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
  public SysIdRoutine buildLeftSysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(ClimbConstants.RAMP_RATE_VOLTS_SYSID).per(Seconds),
            Volts.of(ClimbConstants.DYNAMIC_STEP_VOLTS_SYSID),
            null,
            (state) -> SignalLogger.writeString("leftState", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> climbIO.setLeftVoltage(volts.in(Volts)), null, this));
  }

  /** Build SysId Routine for the Right Motor */
  public SysIdRoutine buildRightSysIdRoutine() {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(ClimbConstants.RAMP_RATE_VOLTS_SYSID).per(Seconds),
            Volts.of(ClimbConstants.DYNAMIC_STEP_VOLTS_SYSID),
            null,
            (state) -> SignalLogger.writeString("rightState", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> climbIO.setRightVoltage(volts.in(Volts)), null, this));
  }
}

/*
 * have one command that runs a manual climb
 *
 * have one command that's automatic that runs to a position
 */
