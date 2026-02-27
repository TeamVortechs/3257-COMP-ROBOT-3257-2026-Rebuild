package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs;

  private final Notifier logger;

  private boolean isLocked = true;

  private double manualLeftSpeed = 0;
  private double manualRightSpeed = 0;

  private double automaticLeftSetpoint = 0;
  private double automaticRightSetpoint = 0;

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
    this.inputs = new ClimbIOInputsAutoLogged();

    // set up logging
    logger =
        new Notifier(
            () -> {
              climbIO.updateInputs(inputs);
              Logger.processInputs("climb", inputs);
            });

    logger.startPeriodic(1 / ClimbConstants.FREQUENCY_HZ);
  }

  @Override
  public void periodic() {}

  public void setVoltage(double voltage) {

    // safety to make sure it doesn't break
    if (isLocked && voltage < 0) {
      voltage = 0;
    }

    climbIO.setVoltage(voltage);
  }

  public void setLocked(boolean locked) {
    isLocked = locked;

    if (isLocked) {
      climbIO.setServo(ClimbConstants.SERVO_CLOSED);
    } else {
      climbIO.setServo(ClimbConstants.SERVO_OPEN);
    }
  }

  // this is a run command because we do a check everytime we run this command
  public Command setVoltageRun(double voltage) {
    return new RunCommand(() -> setVoltage(voltage), this);
  }

  public Command setVoltageInstant(double voltage) {
    return new InstantCommand(() -> setVoltage(voltage), this);
  }

  public Command setLockedInstant(boolean locked) {
    return new InstantCommand(() -> setLocked(locked));
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
        new SysIdRoutine.Mechanism((volts) -> climbIO.setVoltage(volts.in(Volts)), null, this));
  }
}

/*
 * have one command that runs a manual climb
 *
 * have one command that's automatic that runs to a position
 */
