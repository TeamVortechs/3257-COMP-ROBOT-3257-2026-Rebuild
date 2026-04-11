package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class EyeOfTheStorm {
  private final PhotonCamera m_LeftCamera;
  private final PhotonCamera m_RightCamera;
  private final PhotonPoseEstimator m_LeftPhotonPoseEstimator;
  private final PhotonPoseEstimator m_RightPhotonPoseEstimator;
  private final EstimateConsumer estConsumer;

  public EyeOfTheStorm(EstimateConsumer estConsumer) {
    this.estConsumer = estConsumer;
    this.m_LeftCamera = new PhotonCamera(VisionConstants.photonLeftName);
    this.m_RightCamera = new PhotonCamera(VisionConstants.photonRightName);
    this.m_LeftPhotonPoseEstimator =
        new PhotonPoseEstimator(VisionConstants.aprilTagLayout, VisionConstants.robotToPhotonLeft);
    this.m_RightPhotonPoseEstimator =
        new PhotonPoseEstimator(VisionConstants.aprilTagLayout, VisionConstants.robotToPhotonRight);
  }

  public void periodic() {
    this.submitVisionEstimates(m_LeftCamera, m_LeftPhotonPoseEstimator);
    this.submitVisionEstimates(m_RightCamera, m_RightPhotonPoseEstimator);
  }

  private void submitVisionEstimates(PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
    for (var result : camera.getAllUnreadResults()) {
      var visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }

      visionEst.ifPresent(
          est -> {
            var estStdDevs = this.getEstimationStdDevs(est, photonEstimator, result.getTargets());
            estStdDevs.ifPresent(
                stdDevs ->
                    estConsumer.accept(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs));
          });
    }
  }

  private Optional<Matrix<N3, N1>> getEstimationStdDevs(
      EstimatedRobotPose estimatedPose,
      PhotonPoseEstimator photonEstimator,
      List<PhotonTrackedTarget> targets) {

    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;
    var estimatedTranslation = estimatedPose.estimatedPose.toPose2d().getTranslation();

    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }

      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedTranslation);
    }

    if (numTags == 0) {
      return Optional.of(VisionConstants.kSingleTagStdDevs);
    }

    avgDist /= numTags;

    if (numTags > 1) {
      estStdDevs = VisionConstants.kMultiTagStdDevs;
    }

    if (numTags == 1 && avgDist > 4) {
      return Optional.empty();
    }

    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    return Optional.of(estStdDevs);
  }

  @FunctionalInterface
  public interface EstimateConsumer {
    void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
