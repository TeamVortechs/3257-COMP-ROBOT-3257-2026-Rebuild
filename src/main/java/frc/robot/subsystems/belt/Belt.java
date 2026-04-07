package frc.robot.subsystems.belt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase {

  // just here for the logging, done like this so we can put it on a dashboard

  private BeltIO beltIO;
  private BeltIOInputsAutoLogged inputs;

  //  private final Notifier logger;

  /**
   * @param beltIO the hardware interface
   */
  public Belt(BeltIO beltIO) {
    this.beltIO = beltIO;
    this.inputs = new BeltIOInputsAutoLogged();

    // // set up logging
    // logger =
    //     new Notifier(
    //         () -> {
    //           beltIO.updateInputs(inputs);
    //           Logger.processInputs("belt", inputs);
    //         });

    // logger.startPeriodic(1 / BeltConstants.FREQUENCY_HZ);
  }

  @Override
  public void periodic() {
    beltIO.updateInputs(inputs);
    Logger.processInputs("belt", inputs);
  }

  // SUBSYSTEM METHODS

  /**
   * @param speed the voltage percentage, between -1.0 to 1.0
   */
  public void setPercentMotorOutput(double speed) {
    beltIO.setPercentMotorOutput(speed);
  }

  /**
   * @return speed in rotations per second
   */
  public double getSpeed() {
    return beltIO.getSpeed();
  }

  // COMMANDS
  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorOutputCommand(double speed) {
    return new InstantCommand(() -> this.setPercentMotorOutput(speed), this);
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorOutputRunCommand(double speed) {
    return Commands.startRun(() -> this.setPercentMotorOutput(speed), () -> {}, this);
  }

  public Command setPercentMotorOutputRunCommand(double speed, BooleanSupplier run) {
    return Commands.run(
        () -> {
          if (run.getAsBoolean()) {
            this.setPercentMotorOutput(speed);
            return;
          }

          this.setPercentMotorOutput(0);
        },
        this);
  }

  public Command setPercentMotorOutputRunCommandAutoEvent(double speed, BooleanSupplier run) {
    return Commands.run(
        () -> {
          if (run.getAsBoolean()) {
            this.setPercentMotorOutput(speed);
            return;
          }

          this.setPercentMotorOutput(0);
        });
  }
}
