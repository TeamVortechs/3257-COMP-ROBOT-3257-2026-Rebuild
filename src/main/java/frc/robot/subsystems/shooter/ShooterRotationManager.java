package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterRotationManager {
  private Supplier<Pose2d> targetPose;
  private Supplier<Pose2d> robotPose;

  @AutoLogOutput private double distance = 0;

  /**
   * @param targetPose the pose of the area we want to shoot too
   * @param robotPose the pose of the robot
   */
  public ShooterRotationManager(Supplier<Pose2d> targetPose, Supplier<Pose2d> robotPose) {
    this.targetPose = targetPose;
    this.robotPose = robotPose;
  }

  /**
   * Get the distance from the robot pose to the target pose
   *
   * @return the distance in meters
   */
  public double getDistance() {
    distance = targetPose.get().getTranslation().getDistance(robotPose.get().getTranslation());

    return distance;
  }

  /**
   * Get the heading from target pose to the robot pose(FIELD CENTRIC)
   *
   * @return field centric heading
   */
  public Rotation2d getHeading() {
    // makes it so the robot will rotate towards where it is moving when driving to the pose
    Translation2d delta = targetPose.get().getTranslation().minus(robotPose.get().getTranslation());

    Rotation2d heading = new Rotation2d(delta.getX(), delta.getY());
    ;

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
    Rotation2d robotRotation = robotPose.get().getRotation();

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
}
