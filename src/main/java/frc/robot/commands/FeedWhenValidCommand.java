package frc.robot.commands;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterRotationManager;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/*
Names
brief description

Runs the feeder only when the shot is valid
*/
// rename ts
public class FeedWhenValidCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  /** timer since it was last valid. The code waits a certain time before it is ok to shooot */
  private Timer validityTimer;

  private Feeder feeder;

  // controller to give feedback too
  private CommandXboxController controller;

  // these are just used for validity requirements
  private Shooter shooter;
  private ShooterRotationManager shooterRotationManager;
  private Drive drive;

  // used for measuring if we are measuring different chassis speeds(skid) which means the shooter
  // is most likely not accurate
  private BuiltInAccelerometer accelerometer;

  private double accelerometerX = 0;
  private double accelerometerY = 0;

  private double chassisX = 0;
  private double chassisY = 0;

  // here for logging
  private boolean isValid = false;
  private boolean isFeedable = false;

  private boolean speedWithinTolerance = false;
  private boolean isOriented = false;
  private boolean isNotSkidding = false;
  private boolean seesAprilTag = false;

  private BooleanSupplier overrideVision;

  /**
   * @param feeder only mechanism this command contorls
   * @param controller gives feedback
   * @param shooter checks for shooter speed
   * @param shooterRotationManager checks that rotation is on target
   * @param drive checks that there is no skid
   * @param overrideVision wether or not we override vision and acceleration change detection when
   *     feeding.
   */
  public FeedWhenValidCommand(
      Feeder feeder,
      CommandXboxController controller,
      Shooter shooter,
      ShooterRotationManager shooterRotationManager,
      Drive drive,
      BooleanSupplier overrideVision) {
    // addRequirements(null);

    this.feeder = feeder;
    this.controller = controller;

    this.shooter = shooter;
    this.shooterRotationManager = shooterRotationManager;
    this.drive = drive;

    accelerometer = new BuiltInAccelerometer();
    validityTimer = new Timer();

    this.overrideVision = overrideVision;

    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    validityTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // this is done this way to allow loging
    checkCurValidity();
    checkFeedability(isValid);

    log();

    if (!isFeedable) {
      feeder.setSpeed(0);
      controller.setRumble(RumbleType.kBothRumble, 0);
      return;
    }

    feeder.setSpeed(FeederConstants.FEED_POWER);
    // purely sim feedback
    controller.setRumble(RumbleType.kBothRumble, 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setSpeed(0);
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** calculates wether or not it is currently ok to make a shot */
  private void checkCurValidity() {
    speedWithinTolerance = shooter.isOnTarget();
    isOriented = shooterRotationManager.isOriented();

    // slip test
    chassisX = drive.getChassisSpeeds().vxMetersPerSecond;
    chassisY = drive.getChassisSpeeds().vyMetersPerSecond;

    accelerometerX = accelerometer.getX();
    accelerometerY = accelerometer.getY();

    isNotSkidding =
        Math.abs(accelerometerX - chassisX) < ShooterConstants.SKID_THRESHOLD
            && Math.abs(accelerometerY - chassisY) < ShooterConstants.SKID_THRESHOLD;

    // we can also check for jerk here
    // not adding skid/slip test here because this is sim

    // we don't have vision up yet here
    seesAprilTag = true;

    // we don't have an accelerometer in sim
    if (Constants.currentMode == Mode.SIM) {
      isNotSkidding = true;
    }

    // simpler check if we don't care abotu vision for whatever reason
    if (overrideVision.getAsBoolean()) {
      isValid = speedWithinTolerance && isOriented;

      // do this if we care abotu vision
    } else {
      isValid = speedWithinTolerance && isOriented && isNotSkidding && seesAprilTag;
    }
  }

  /** takes debounce time into consideration to see if we should we feed right away or still wait */
  private void checkFeedability(boolean isValid) {

    if (!isValid) {
      validityTimer.restart();
      isFeedable = false;
      return;
    }

    isFeedable =
        validityTimer.hasElapsed(ShooterConstants.VALIDITY_DEBOUNCE_TIME_SEC)
            ||
            // this way there won't be a wait time if the user needs it shot now
            overrideVision.getAsBoolean();
  }

  private void log() {
    Logger.recordOutput("FeedWhenValidCommand/timer", validityTimer.get());
    Logger.recordOutput("FeedWhenValidCommand/isCurrentlyValid", isValid);
    Logger.recordOutput("FeedWhenValidCommand/isFeeding", isFeedable);
    Logger.recordOutput("FeedWhenValidCommand/ShooterSpeedWithinTolerance", speedWithinTolerance);
    Logger.recordOutput("FeedWhenValidCommand/IsOrientedCorrectly", isOriented);
    Logger.recordOutput("FeedWhenValidCommand/isNotSkidding", isNotSkidding);
    Logger.recordOutput("FeedWhenValidCommand/accelerometerX", accelerometerX);
    Logger.recordOutput("FeedWhenValidCommand/accelerometerY", accelerometerY);
    Logger.recordOutput("FeedWhenValidCommand/chassisX", chassisX);
    Logger.recordOutput("FeedWhenValidCommand/chassisY", chassisY);
    Logger.recordOutput("FeedWhenValidCommand/seesAprilTag", seesAprilTag);
  }
}
