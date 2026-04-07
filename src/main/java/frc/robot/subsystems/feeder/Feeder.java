package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private FeederIO feederIO;
  private FeederIOInputsAutoLogged inputs;

  // private final Notifier hardwareLogger;

  private final FeederValidityContainer feederValidityContainer;

  // here so we can log it
  // caching this value and calculating it in periodic so we can log it for the driver
  /**
   * @param feederIO the hardware interface
   */
  public Feeder(FeederIO feederIO, FeederValidityContainer validityContainer) {
    this.feederIO = feederIO;
    this.inputs = new FeederIOInputsAutoLogged();

    this.feederValidityContainer = validityContainer;

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
   * @return speed in Rotations per second
   */
  public double getSpeed() {
    return feederIO.getSpeed();
  }

  public double getTargetSpeed() {
    return feederIO.getTargetSpeed();
  }

  public boolean isValidToFeed() {
    return feederValidityContainer.isValid();
  }

  /**
   * @return target speed
   */

  // HELPER METHODS

  // COMMANDS
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
            this.setPercentMotorOutput(percentage);
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
            this.setPercentMotorOutput(percentage);
          } else {
            this.setPercentMotorOutput(0);
          }
        });
  }

  /** can swap this for the other command in auto */
  public Command feedWhenShooterIsRevvedCommand(double percentage) {
    return Commands.run(
        () -> {
          if (feederValidityContainer.shooterIsRevved()) {
            this.setPercentMotorOutput(percentage);
          } else {
            this.setPercentMotorOutput(0);
          }
        },
        this);
  }
}
