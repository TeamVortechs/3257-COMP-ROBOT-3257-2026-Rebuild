package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class AlliancePoseManager {

  private static Alliance alliance = Alliance.Blue;

  public static final Supplier<Pose2d> PASSING_POSE_DOWN =
      alliancePoseSupplier(
          DriveConstants.PASSING_POSE_DOWN_BLUE, DriveConstants.PASSING_POSE_DOWN_RED);

  public static final Supplier<Pose2d> PASSING_POSE_UP =
      alliancePoseSupplier(DriveConstants.PASSING_POSE_UP_BLUE, DriveConstants.PASSING_POSE_UP_RED);

  public static final Supplier<Pose2d> GOAL_POSE =
      alliancePoseSupplier(DriveConstants.GOAL_POSE_BLUE, DriveConstants.GOAL_POSE_RED);

  public static void update() {

    Optional<Alliance> curAlliance = DriverStation.getAlliance();

    if (curAlliance.isEmpty() || curAlliance.get() == Alliance.Blue) {
      alliance = Alliance.Blue;
    } else {
      alliance = Alliance.Red;
    }
  }

  private static Supplier<Pose2d> alliancePoseSupplier(Pose2d bluePose, Pose2d redPose) {

    return () -> {
      if (alliance == Alliance.Blue) {
        return bluePose;
      } else {
        return redPose;
      }
    };
  }
}
