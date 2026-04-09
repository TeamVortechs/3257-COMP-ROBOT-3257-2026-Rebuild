package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FeederConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private FeederIO feederIO;
  private FeederIOInputsAutoLogged inputs;

  // private final Notifier hardwareLogger;

  private BooleanSupplier shooterOnTarget;
  private BooleanSupplier isOriented;

  // here so we can log it
  // caching this value and calculating it in periodic so we can log it for the driver
  /**
   * @param feederIO the hardware interface
   */
  public Feeder(FeederIO feederIO, BooleanSupplier shooterOnTarget, BooleanSupplier isOriented) {
    this.feederIO = feederIO;
    this.inputs = new FeederIOInputsAutoLogged();

    this.shooterOnTarget = shooterOnTarget;
    this.isOriented = isOriented;
    // hardwareLogger =
    //     new Notifier(
    //         () -> {
    //           feederIO.updateInputs(inputs);
    //           Logger.processInputs("feeder", inputs);
    //         });

    // hardwareLogger.startPeriodic(1 / FeederConstants.SUBSYSTEM_LOGGING_FREQUENCY_HERTZ);
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(inputs);
    Logger.processInputs("feeder", inputs);
  }

  // SUBSYSTEM METHODS

  /**
   * @param speed the percentage of battery voltage the belts will take. -1 to 1
   */
  public void setPercentMotorOutput(double speed) {
    feederIO.setPercentMotorOutput(speed);
  }

  /**
   * Sets feeder speed
   *
   * @param speed Speed in RPS
   */
  public void setSpeed(double speed) {
    feederIO.setSpeed(speed);
  }
  /**
   * @return speed in Rotations per second
   */
  public double getSpeed() {
    return feederIO.getSpeed();
  }

  public double getTargetSpeed() {
    return feederIO.getTargetSpeed();
  }

  public boolean isValidToFeed() {
    return isOriented.getAsBoolean() && shooterOnTarget.getAsBoolean();
  }

  /**
   * @return target speed
   */

  // HELPER METHODS

  // COMMANDS

  public Command setSpeedCommand(double speed) {
    return Commands.startRun(
        () -> {
          feederIO.setSpeed(speed);
        },
        () -> {},
        this);
  }

  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param percentage the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorCommand(double percentage) {
    return new InstantCommand(() -> this.setPercentMotorOutput(percentage));
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param percentage the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorRunCommand(double percentage) {
    return Commands.startRun(() -> this.setPercentMotorOutput(percentage), () -> {}, this);
  }

  /**
   * feeds when the drive, limelight, and shooter are ontarget
   *
   * @param percentage the percentage of battery to supply to the feeder
   * @return the command
   */
  public Command feedWhenValidRunCommand(double percentage) {
    return Commands.run(
        () -> {
          if (isValidToFeed()) {
            this.setSpeed(98);
          } else {
            this.setPercentMotorOutput(0);
          }
        },
        this);
  }

  /**
   * feeds when the drive, limelight, and shooter are ontarget. USED DURIGN AUTO SINCE THIS DOESN'T
   * HAVE REQUIREMENTS
   *
   * @param percentage the percentage of battery to supply to the feeder
   * @return the command
   */
  public Command feedWhenValidRunCommandAutoEvent(double percentage) {
    return Commands.run(
        () -> {
          if (isValidToFeed()) {
            this.setSpeed(98);
          } else {
            this.setPercentMotorOutput(0);
          }
        });
  }

  /** can swap this for the other command in auto */
  public Command feedWhenShooterIsRevvedCommand(double percentage) {
    return Commands.run(
        () -> {
          if (shooterOnTarget.getAsBoolean()) {
            this.setSpeed(98);

          } else {
            this.setPercentMotorOutput(0);
          }
        },
        this);
  }

  // the constants here should probably be more and move but that's later when this is transferred
  // to the right project
  // add this to the robot class or this won't work: SignalLogger.setPath("/media/sda1/");
  /**
   * Gets the system identification routine for this specific subsystem
   *
   * @return the sysid routine
   */
  public SysIdRoutine BuildSysIdRoutine() {

    SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(FeederConstants.RAMP_RATE_VOLTS_SYSID)
                    .per(Seconds), // Ramp Rate in Volts / Seconds
                Volts.of(FeederConstants.DYNAMIC_STEP_VOLTS_SYSID), // Dynamic Step Voltage
                null, // Use default timeout (10 s)
                (state) ->
                    SignalLogger.writeString(
                        "state", state.toString()) // Log state with Phoenix SignalLogger class
                ),
            new SysIdRoutine.Mechanism(
                (volts) -> feederIO.setVoltage(volts.in(Volts)), null, this));
    return m_SysIdRoutine;
  }
}
