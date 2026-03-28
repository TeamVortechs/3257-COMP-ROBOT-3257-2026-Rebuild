package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/*
Names
brief description
 */
public class PathfindToPoseCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Drivetrain drive;

  private Supplier<Pose2d> targetPoseSupplier;

  // PIDController
  private final PIDController translationController =
      new PIDController(
          Constants.DriveConstants.TRANS_KP,
          Constants.DriveConstants.TRANS_KI,
          Constants.DriveConstants.TRANS_KD);

  private final PIDController thetaController =
      new PIDController(
          Constants.DriveConstants.ANGLE_KP,
          Constants.DriveConstants.ANGLE_KI,
          Constants.DriveConstants.ANGLE_KD);

  private final double translationTolerance = Constants.DriveConstants.TRANS_TOLERANCE;
  private final double rotationTolerance = Constants.DriveConstants.ORIENTATION_TOLERANCE;

  private final boolean endOnTarget;
  private Consumer<Boolean> onTarget = null;

  // variable changing velocities
  private double xVelocity = 0;
  private double yVelocity = 0;
  private double thetaVelocity = 0;

  private double thetaDistance = 0;
  private double translationDistanceX = 0;
  private double translationDistanceY = 0;
  private double totalDist = 0;

  // max time command runs for, starts on init
  private Timer timer;
  private double timeout = 10; // times out after timer reaches this time

  /**
   * @param drive
   * @param targetPose
   * @param endOnTarget
   */
  public PathfindToPoseCommand(Drivetrain drive, Supplier<Pose2d> targetPose, boolean endOnTarget) {
    addRequirements(drive);
    this.drive = drive;

    this.targetPoseSupplier = targetPose;

    this.endOnTarget = endOnTarget;

    timer = new Timer();
    timer.reset();

    // record outputs
    Logger.recordOutput("DriveToObject/PathfindxVelocity", xVelocity);
    Logger.recordOutput("DriveToObject/PathfindyVelocity", yVelocity);
    Logger.recordOutput("DriveToObject/PathfindthetaVelocity", thetaVelocity);
    Logger.recordOutput("DriveToObject/PathfindtranslationDistanceX", translationDistanceX);
    Logger.recordOutput("DriveToObject/PathfindtranslationDistanceY", translationDistanceY);
    Logger.recordOutput("DriveToObject/PathfindthetaDistanceRad", thetaDistance);
    Logger.recordOutput("DriveToObject/WithinTolerance", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();

    // (Optional but good): allow wrapping for theta
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // obtains this for alliance multipler + field relative conversions
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // calculates the speeds needed(field relative) to move the robot towards the target
    ChassisSpeeds fieldRelativeSpeeds = calculateFieldRelativeSpeedsToTarget(isFlipped);

    // if the interupter is true we shouldn't move so I'll just make chassis speeds zero

    // if the contorller is greater than the deadband use the controller for translation instead of
    // the given speeds

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            isFlipped
                ? drive.getPose().getRotation().plus(new Rotation2d(Math.PI))
                : drive.getPose().getRotation()));
    // record outputs
    Logger.recordOutput("DriveToObject/xVelocity", xVelocity);
    Logger.recordOutput("DriveToObject/yVelocity", yVelocity);
    Logger.recordOutput("DriveToObject/thetaVelocity", thetaVelocity);
    Logger.recordOutput("DriveToObject/PathfindtranslationDistanceX", translationDistanceX);
    Logger.recordOutput("DriveToObject/PathfindtranslationDistanceY", translationDistanceY);
    Logger.recordOutput("DriveToObject/thetaDistanceRad", thetaDistance);
    Logger.recordOutput("DriveToObject/totalDist", totalDist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // calculates wether or not the robot is within goal tolerance
    boolean atGoal =
        Math.abs(translationDistanceX) < translationTolerance
            && Math.abs(translationDistanceY) < translationTolerance
            && Math.abs(thetaDistance) < rotationTolerance;

    // updates the "ontarget" boolean consumer in case extraneous systems need it
    if (onTarget != null) {
      if (atGoal) {
        onTarget.accept(true);
      } else {
        onTarget.accept(false);
      }
    }

    Logger.recordOutput("DriveToObject/WithinTolerance", atGoal);

    // if the command is sent to end on target then end if reached timeout or it is at goal
    if (endOnTarget) {
      return atGoal || timer.hasElapsed(timeout);
    }

    return false;
  }

  /**
   * helper method to calculate the field relative speeds needed to drive the robot to the object
   *
   * @param isFlipped
   * @return
   */
  private ChassisSpeeds calculateFieldRelativeSpeedsToTarget(boolean isFlipped) {
    // obtain target/current poses
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    // calculate distances
    translationDistanceX = targetPose.getX() - currentPose.getX();
    translationDistanceY = targetPose.getY() - currentPose.getY();
    thetaDistance = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

    // makeks the robot always choose the shortest rotational path
    if (thetaDistance > Math.PI) {
      thetaDistance = Math.PI * 2 - thetaDistance;
    }

    int allianceMultiplier = !isFlipped ? 1 : -1;

    // calculate velocitie
    xVelocity =
        allianceMultiplier * translationController.calculate(currentPose.getX(), targetPose.getX());
    yVelocity =
        allianceMultiplier * translationController.calculate(currentPose.getY(), targetPose.getY());

    thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // restrict velocity to within top speeds, implemented bc trapezoidal profile didn't work
    xVelocity =
        MathUtil.clamp(
            xVelocity,
            -Constants.DriveConstants.TRANS_TOP_SPEED,
            Constants.DriveConstants.TRANS_TOP_SPEED);

    yVelocity =
        MathUtil.clamp(
            yVelocity,
            -Constants.DriveConstants.TRANS_TOP_SPEED,
            Constants.DriveConstants.TRANS_TOP_SPEED);

    thetaVelocity =
        MathUtil.clamp(
            thetaVelocity,
            -Constants.DriveConstants.ANGLE_MAX_VELOCITY,
            Constants.DriveConstants.ANGLE_MAX_VELOCITY);

    return new ChassisSpeeds(xVelocity, yVelocity, thetaVelocity);
  }
}
