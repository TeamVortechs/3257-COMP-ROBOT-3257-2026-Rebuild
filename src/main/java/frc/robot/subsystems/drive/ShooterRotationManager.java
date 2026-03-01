package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** change this so it only logs hwen it's getting consumed */
/*
 * THIS CODE IS REALLY BAD, FOR NOW IT'S GONNA STAY LIKE THIS.
 */
public class ShooterRotationManager {
  private Supplier<Pose2d> targetPose;
  private Drive drive;

  private ProfiledPIDController angle_controller = DriveConstants.ANGLE_CONTROLLER;

  // private final Notifier logger;

  // used to stop recursive calls with get dsitance and get effective target. This is really jank
  // but it's the only thing I could think of without a  rewrite

  /**
   * @param targetPose the pose of the area we want to shoot too
   * @param drive the pose of the robot
   */
  public ShooterRotationManager(Supplier<Pose2d> targetPose, Drive drive, double loggingFrequency) {
    this.targetPose = targetPose;
    this.drive = drive;

    // set up logging
    // logger =
    //     new Notifier(
    //         () -> {
    //           log();
    //         });
  }

  public void startLogThread() {
    // logger.startPeriodic(1 / DriveConstants.SHOOTER_ROTATION_MANAGER_LOGGING_FREQUENCY);
  }

  /*
   * Logs all values, should be called a small amount of times per second
   */
  private void log() {
    // all of these methods automatically log values

    // logs is oriented, also calls get heaidng, not needed rn cus this is called in feeder
    // isOriented();

    // logs distance
    getDistance();
  }

  /**
   * Get the distance from the robot pose to the target pose
   *
   * @return the distance in meters
   */
  public double getDistance() {
    double distance =
        getEffectiveTarget().getTranslation().getDistance(getPoseAtRelease().getTranslation());

    // logs so we have good logging of this when it's important
    // Logger.recordOutput("ShooterRotationManager/Distance", distance);

    return distance;
  }

  public double getRotationFeedbackOverride() {

    double targetRadians = this.getHeading().getRadians();
    double currentRadians = drive.getPose().getRotation().getRadians();

    double rotationSpeed = angle_controller.calculate(currentRadians, targetRadians);

    // Logger.recordOutput("ShooterRotationManager/RotationFeedbackOverride", rotationSpeed);

    return rotationSpeed;
    // return .1;
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
        getEffectiveTarget().getTranslation().minus(getPoseAtRelease().getTranslation());

    Rotation2d heading = new Rotation2d(delta.getX(), delta.getY());

    // Logger.recordOutput("ShooterRotationManager/TargetHeading", heading);

    return heading;
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

    // System.out.println("logging is oritented in shooter rot manager");

    // it's possible we could make tolerance a function of distance if it is a limiting factor
    boolean onTarget =
        Math.abs(error.getRadians()) < Constants.DriveConstants.ORIENTATION_TOLERANCE;

    Logger.recordOutput("ShooterRotationManager/IsOriented", onTarget);

    return onTarget;
  }

  /**
   * gets the predicted pose after a k amount of seconds. This was we can adjust for robot lag and
   * shooting lag
   *
   * @return
   */
  public Pose2d getPoseAtRelease() {
    Pose2d firstPose = drive.getPose();

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    // this scaling factor is a constnat we'll just need to test for. We can change it depending on
    // if the shot is compensation to much or not enough. We can also make it zero to remove it

    double dt = DriveConstants.KRELEASE_POSE_PREDICTION_SEC;

    Rotation2d predictedRot =
        firstPose.getRotation().plus(new Rotation2d(fieldSpeeds.omegaRadiansPerSecond * dt));

    Pose2d updatedPose =
        new Pose2d(
            firstPose.getX() + fieldSpeeds.vxMetersPerSecond * dt,
            firstPose.getY() + fieldSpeeds.vyMetersPerSecond * dt,
            predictedRot);

    // Logger.recordOutput("ShooterRotationManager/PoseAtRelease", updatedPose);

    return updatedPose;
  }

  public Pose2d getEffectiveTarget() {

    ChassisSpeeds fieldSpeedsChassis =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    Translation2d fieldSpeeds =
        new Translation2d(
            fieldSpeedsChassis.vxMetersPerSecond, fieldSpeedsChassis.vyMetersPerSecond);

    Translation2d fieldSpeedsNormalized = fieldSpeeds.div(fieldSpeeds.getNorm());

    double upperBound = 0;
    double lowerBound = -30;
    double testPoint = 0;

    double error = 10;

    int logNum = 0;

    // binary search
    while (error > DriveConstants.SHOOT_ON_MOVE_TOLERANCE) {

      testPoint = lowerBound + (upperBound - lowerBound) / 2;

      Translation2d testPose = testDistance(testPoint, fieldSpeeds);

      // put it onto a line so that we can do binary search logic
      Translation2d difference = targetPose.get().getTranslation().minus(testPose);
      double differenceOnLine = difference.dot(fieldSpeedsNormalized);

      error = Math.abs(differenceOnLine);

      // binary search stuff
      if (differenceOnLine >= 0) {
        lowerBound = testPoint;
      } else {
        upperBound = testPoint;
      }

      // System.out.println("running loop " + logNum);

      // Logger.recordOutput("test/testPose" + logNum, new Pose2d(testPose, new Rotation2d()));
      // Logger.recordOutput("test/differenceOnLine" + logNum, differenceOnLine);
      // Logger.recordOutput("test/testVal" + logNum, testPoint);
      // Logger.recordOutput("test/testPose" + logNum, new Pose2d(testPose, new Rotation2d()));

      logNum++;

      if (logNum > 20) {
        break;
      }
    }

    // Logger.recordOutput("test/loopAmount", logNum);
    // Logger.recordOutput("test/targetPose", getTargetPose(testPoint, fieldSpeedsNormalized));

    return getTargetPose(testPoint, fieldSpeedsNormalized);
  }

  /**
   * tests a difference along the line. Returns the pose where we think the note will land given our
   * model
   *
   * @param distanceAlongLine
   * @param loggingPrefix
   * @param fieldSpeeds
   * @return
   */
  public Translation2d testDistance(double distanceAlongLine, Translation2d fieldSpeeds) {

    Translation2d targetTranslation =
        getTargetPose(distanceAlongLine, fieldSpeeds).getTranslation();

    double distance = drive.getPose().getTranslation().getDistance(targetTranslation);

    Translation2d landingTranslation =
        targetTranslation.plus(fieldSpeeds.times(DriveConstants.getTimeInAir(distance)));

    return landingTranslation;
  }

  /**
   * gets a pose along hte line
   *
   * @param distanceAlongLine
   * @param fieldSpeeds
   * @return
   */
  public Pose2d getTargetPose(double distanceAlongLine, Translation2d fieldSpeeds) {
    Translation2d fieldSpeedsNormalized = fieldSpeeds.div(fieldSpeeds.getNorm());
    Translation2d offset = fieldSpeedsNormalized.times(distanceAlongLine);
    Translation2d targetTranslation = targetPose.get().getTranslation().plus(offset);

    return new Pose2d(targetTranslation, new Rotation2d());
  }
}

/**
 * ts is only useful if we get a turret later
 *
 * <p>public Rotation2d getRobotRelativeRotation() { Rotation2d heading = getHeading(); Rotation2d
 * robotRotation = getPoseAtRelease().getRotation();
 *
 * <p>return heading.minus(robotRotation); }
 *
 * <p>public static double convertRotationToMotorVal(Rotation2d rotation) { return 0; }
 *
 * <p>public static double convertDistToShooterAngle(double distance) { return 0; }
 */

 /*
 *     Pose2d firstPose = targetPose.get();

   ChassisSpeeds fieldSpeeds =
       ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

   // calculate distance without effective target because otherwise ti would breka the calculation
   double distance =
       targetPose.get().getTranslation().getDistance(drive.getPose().getTranslation());

   // this scaling factor is a constnat we'll just need to test for. We can change it depending on
   // if the shot is compensation to much or not enough. We can also make it zero to remove it
   // it is negative to make it minus in the final equation
   double dt = DriveConstants.TIME_IN_AIR(distance);
   Pose2d updatedTarget =
       new Pose2d(
           firstPose.getX() - fieldSpeeds.vxMetersPerSecond * dt,
           firstPose.getY() - fieldSpeeds.vyMetersPerSecond * dt,
           firstPose.getRotation());

   // Logger.recordOutput("ShooterRotationManager/EffectiveTarget", updatedTarget);

   return updatedTarget;
 */
