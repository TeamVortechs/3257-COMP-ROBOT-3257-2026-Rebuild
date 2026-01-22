package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/*
 * THIS CODE IS REALLY BAD, FOR NOW IT'S GONNA STAY LIKE THIS.
 */
public class ShooterRotationManager {
  private Supplier<Pose2d> targetPose;
  private Drive drive;

  @AutoLogOutput private double distance = 0;
  @AutoLogOutput private double currentRadians;
  @AutoLogOutput private double targetRadians;
  @AutoLogOutput private double radiansDifference;
  @AutoLogOutput private boolean onTarget = false;
  @AutoLogOutput private Pose2d unFilteredTargetPose = new Pose2d();
  @AutoLogOutput private Pose2d unFilteredCurrentPose = new Pose2d();

  /**
   * @param targetPose the pose of the area we want to shoot too
   * @param drive the pose of the robot
   */
  public ShooterRotationManager(Supplier<Pose2d> targetPose, Drive drive) {
    this.targetPose = targetPose;
    this.drive = drive;
  }

  // logs all of the values from this. Should be called repeatedly
  public void log() {
    // all of these methods automatically log values
    currentRadians = drive.getRotation().getRadians();
    targetRadians = getHeading().getRadians();
    getDistance();
    getHeading();
    isOriented();
  }

  /**
   * Get the distance from the robot pose to the target pose
   *
   * @return the distance in meters
   */
  public double getDistance() {
    distance =
        getEffectiveTarget().getTranslation().getDistance(getPoseAtRelease().getTranslation());

    return distance;
  }

  /**
   * Get the heading from robot pose to target pose(FIELD CENTRIC)
   *
   * @return field centric heading
   */
  public Rotation2d getHeading() {
    // makes it so the robot will rotate towards where it is moving when driving to the pose
    // im not sure if this should be current or predicted drive, I'll ask
    Translation2d delta =
        getEffectiveTarget().getTranslation().minus(drive.getPose().getTranslation());

    Rotation2d heading = new Rotation2d(delta.getX(), delta.getY());

    getDistance();

    return heading;
  }

  /**
   * Robot relative rotation to make the object point towards the target pose
   *
   * @return robot relative rotation
   */
  public Rotation2d getRobotRelativeRotation() {
    Rotation2d heading = getHeading();
    Rotation2d robotRotation = getPoseAtRelease().getRotation();

    return heading.minus(robotRotation);
  }

  /**
   * converts the rotation to an encoder value on the motor so we can rotate it towards it(MAYBE
   * MOVE INTO MOTOR CLASS)
   *
   * @param rotation the rotation
   * @return the encoder value to aim towards in the PID loop
   */
  public static double convertRotationToMotorVal(Rotation2d rotation) {
    return 0;
  }

  /**
   * converts the distance into arm elevation to shoot towards it
   *
   * @param distance the distance in meters
   * @return the arm encoder value to aim towards
   */
  public static double convertDistToShooterAngle(double distance) {
    return 0;
  }

  /**
   * converts the distance into arm elevation to shoot towards it
   *
   * @param distance the distance in meters
   * @return the arm encoder value to aim towards
   */
  public boolean isOriented() {

    // im not sure if this should be current or predicted drive, I'll ask
    Rotation2d error = getHeading().minus(drive.getRotation());

    radiansDifference = error.getRadians();

    // it's possible we could make tolerance a function of distance if it is a limiting factor
    onTarget = Math.abs(error.getRadians()) < Constants.ShooterConstants.ORIENTATION_TOLERANCE;

    return onTarget;
  }

  /**
   * gets the predicted pose after a k amount of seconds. This was we can adjust for robot lag and
   * shooting lag
   *
   * @return
   */
  @AutoLogOutput
  public Pose2d getPoseAtRelease() {
    Pose2d firstPose = drive.getPose();

    // logs the unfiltered pose so we can see the difference
    unFilteredCurrentPose = firstPose;

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    // this scaling factor is a constnat we'll just need to test for. We can change it depending on
    // if the shot is compensation to much or not enough. We can also make it zero to remove it

    double dt = ShooterConstants.KRELEASE_POSE_PREDICTION_SEC;

    Rotation2d predictedRot =
        firstPose.getRotation().plus(new Rotation2d(fieldSpeeds.omegaRadiansPerSecond * dt));

    Pose2d updatedPose =
        new Pose2d(
            firstPose.getX() + fieldSpeeds.vxMetersPerSecond * dt,
            firstPose.getY() + fieldSpeeds.vyMetersPerSecond * dt,
            predictedRot);

    return updatedPose;
  }

  @AutoLogOutput
  public Pose2d getEffectiveTarget() {
    Pose2d firstPose = targetPose.get();

    // logs the unfiltered pose so we can see the difference
    unFilteredTargetPose = firstPose;

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    // this scaling factor is a constnat we'll just need to test for. We can change it depending on
    // if the shot is compensation to much or not enough. We can also make it zero to remove it
    // it is negative to make it minus in the final equation
    double dt = ShooterConstants.KFLIGHT_COMPENSATION_SEC;
    Pose2d updatedTarget =
        new Pose2d(
            firstPose.getX() - fieldSpeeds.vxMetersPerSecond * dt,
            firstPose.getY() - fieldSpeeds.vyMetersPerSecond * dt,
            firstPose.getRotation());

    return updatedTarget;
  }
}
