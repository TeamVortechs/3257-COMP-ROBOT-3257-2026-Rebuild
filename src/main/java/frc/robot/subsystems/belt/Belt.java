package frc.robot.subsystems.belt;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.BeltConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase {

  // just here for the logging, done like this so we can put it on a dashboard
  @AutoLogOutput(key = "Belt/BeltSpeed")
  private double speed = 0;

  @AutoLogOutput(key = "Belt/BeltTargetSpeed")
  private double targetSpeed = 0;

  @AutoLogOutput(key = "Belt/IsOnTarget")
  private boolean isOnTarget = false;

  private BeltIO beltIO;
  private BeltIOInputsAutoLogged inputs;

  /**
   * @param beltIO the hardware interface
   */
  public Belt(BeltIO beltIO) {
    this.beltIO = beltIO;
    this.inputs = new BeltIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    beltIO.updateInputs(inputs);
    Logger.processInputs("belt", inputs);

    // calculate speed that automatically updates with distance
    // automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());

    targetSpeed = getSpeedTarget();
    speed = beltIO.getSpeed();
    isOnTarget = isOnTarget();

    beltIO.setSpeed(targetSpeed);
  }

  // SUBSYSTEM METHODS

  /**
   * @param speed the speed the motor will pid too
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * @return the target speed that the flywheel is pid'ing to. Can be the manual or the automatic
   *     calculated speed
   */
  public double getSpeedTarget() {
    return speed;
  }

  /**
   * @return wether the speed is the target speed
   */
  public boolean isOnTarget() {
    return beltIO.isOnTarget();
  }

  // COMMANDS
  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setSpeedCommand(double speed) {
    return new InstantCommand(() -> this.setSpeed(speed));
  }

  /**
   * sets the target speed command then ends when it reaches that speed
   *
   * @param speed the speed it gets set to
   * @return the finished command
   */
  public Command setSpeedCommandConsistentEnd(double speed) {
    return new InstantCommand(() -> this.setSpeed(speed))
        .andThen(new WaitUntilCommand(() -> this.isOnTarget()));
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setSpeedRunCommand(double speed) {
    return Commands.run(() -> this.setSpeed(speed), this);
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
                Volts.of(BeltConstants.RAMP_RATE_VOLTS_SYSID)
                    .per(Seconds), // Ramp Rate in Volts / Seconds
                Volts.of(BeltConstants.DYNAMIC_STEP_VOLTS_SYSID), // Dynamic Step Voltage
                null, // Use default timeout (10 s)
                (state) ->
                    SignalLogger.writeString(
                        "state", state.toString()) // Log state with Phoenix SignalLogger class
                ),
            new SysIdRoutine.Mechanism((volts) -> beltIO.setVoltage(volts.in(Volts)), null, this));
    return m_SysIdRoutine;
  }

  // this shoudl be in a helper method or somewhere in robot container
  /**
   * Binds the sys id routine to the controller so we can use it
   *
   * <p>pov up: dynamic forward pov down: dynamic backwards pov right: quasistatic forward pov left:
   * quasistatic reverse
   *
   * @param controller the controller this binds to(recommended to use a high id controller to
   *     prevent mishaps, id 2-3)
   * @param sysIdRoutine the routine that this controller will activate
   */
  public void configureSysIdBindings(CommandXboxController controller, SysIdRoutine sysIdRoutine) {
    controller.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    controller.povDown().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    controller.povRight().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    controller.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));
  }
}
