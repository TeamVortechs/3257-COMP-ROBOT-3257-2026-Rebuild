package frc.robot.util.FieldMovement;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// extended version of the pose estimator that the drivetrain uses. For now this doesn't do anthing
// special but later if we wanted to upgrade the pose system we would just need to add functionality
// to this one
public class VortechsPoseEstimator extends SwerveDrivePoseEstimator {

  // this is to check if the robot actually moves. I don't think we need this right now
  // private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  public VortechsPoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

  // commenting this out for now bc idk if we need it
  // this just changes the code to not update the pose if the robot detects no change in
  // acceleration, pretty simple change but I think it could be pretty good.
  // @Override
  // public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle,
  // SwerveModulePosition[] wheelPositions) {
  //   //makes a new cur pose so I can make changes ot it how I like
  //   Pose2d curPose = new Pose2d(getEstimatedPosition().getX(), getEstimatedPosition().getY(),
  // getEstimatedPosition().getRotation());
  //   Pose2d nextPose = super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);

  //   //0.1 needs to be changed.
  //   if(accelerometer.getX() + accelerometer.getY() < 0.1) {
  //     //rolls back the pose if it detects that nothing changed in the pose
  //     resetPose(curPose);
  //   }

  //   return getEstimatedPosition();
  // }
}
