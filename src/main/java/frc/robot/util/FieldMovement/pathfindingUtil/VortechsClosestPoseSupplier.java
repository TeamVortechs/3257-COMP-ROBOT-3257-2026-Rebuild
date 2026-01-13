package frc.robot.util.FieldMovement.pathfindingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.Supplier;

public class VortechsClosestPoseSupplier {
  private List<Pose2d> targetPoses;
  private Supplier<Pose2d> curPose;

  public VortechsClosestPoseSupplier(List<Pose2d> targetPoses, Supplier<Pose2d> curPose) {
    this.targetPoses = targetPoses;
    this.curPose = curPose;
  }

  public Pose2d getClosestPose() {

    double lowestDistance = Double.MAX_VALUE;

    Pose2d lowestDistPose = new Pose2d();

    for (Pose2d testPose : targetPoses) {
      double distance = testPose.getTranslation().getDistance(curPose.get().getTranslation());

      if (distance < lowestDistance) {
        lowestDistPose = testPose;
        lowestDistance = distance;
      }
    }

    return lowestDistPose;
  }
}
