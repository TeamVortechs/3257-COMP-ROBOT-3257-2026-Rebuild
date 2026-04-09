// // Copyright (c) 2021-2026 Littleton Robotics
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by a BSD
// // license that can be found in the LICENSE file
// // at the root directory of this project.

// package frc.robot.subsystems.vision;

// import static frc.robot.subsystems.vision.VisionConstants.*;

// import com.ctre.phoenix6.Utils;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.Arrays;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;

// public class VisionLogless extends SubsystemBase {
//   private final VisionConsumer consumer;

//   private final PowerModuleIO powerModuleIO;
//   private PowerModuleIOInputsAutoLogged powerModuleIOInputsAutoLogged;
//   protected final PhotonCamera photonCameraLeft;
//   protected final PhotonCamera photonCameraRight;
//   private final PhotonPoseEstimator leftEstimator;
//   private final PhotonPoseEstimator rightEstimator;

//   public VisionLogless(VisionConsumer consumer, PowerModuleIO powerModuleIO) {
//     this.consumer = consumer;
//     this.powerModuleIO = powerModuleIO;
//     powerModuleIOInputsAutoLogged = new PowerModuleIOInputsAutoLogged();

//     this.photonCameraLeft = new PhotonCamera(photon0Name);
//     this.photonCameraRight = new PhotonCamephotonRightNameame);
//     this.leftEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToPhotonLeft);
//     this.rightEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToPhotonRight);
//   }

//   @Override
//   public void periodic() {
//     updateLeftCam();
//     updateRightCam();
//     // powerModuleIO.updateInputs(powerModuleIOInputsAutoLogged);
//   }

//   //   boolean rejectPose =
//   //     observation.tagCount() == 0 // Must have at least one tag
//   //         || (observation.tagCount() == 1
//   //             && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
//   //         || Math.abs(observation.pose().getZ())
//   //             > maxZError // Must have realistic Z coordinate

//   //         // Must be within the field boundaries
//   //         || observation.pose().getX() < 0.0
//   //         || observation.pose().getX() > aprilTagLayout.getFieldLength()
//   //         || observation.pose().getY() < 0.0
//   //         || observation.pose().getY() > aprilTagLayout.getFieldWidth();

//   // // if we're in testing mode, accept poses that extend out of the field boundaries but still
//   // // use the basic filtering
//   // // use "RobotPosesAccepted or RobotPosesRejected" in AdvantageScope to see which ones are
//   // // good in normal operation
//   // if (DriverStation.isTest()) {
//   //   rejectPose =
//   //       observation.tagCount() == 0 // Must have at least one tag
//   //           || (observation.tagCount() == 1
//   //               && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
//   //           || Math.abs(observation.pose().getZ())
//   //               > maxZError; // Must have realistic Z coordinate
//   // }

//   private void updateLeftCam() {
//     if (photonCameraLeft.isConnected()) {
//       for (var result : photonCameraLeft.getAllUnreadResults()) {
//         var poseEst =
//             result.getMultiTagResult().isPresent()
//                 ? leftEstimator.estimateCoprocMultiTagPose(result)
//                 : leftEstimator.estimateLowestAmbiguityPose(result);
//         if (poseEst.isPresent() // if there IS an estimate
//             && (result.multitagResult.isPresent()
//                 || Arrays.binarySearch(bannedTags, result.targets.get(0).fiducialId)
//                     < 0) // and it's either multitag or not banned
//             && result.getBestTarget().poseAmbiguity < maxAmbiguity // and low ambiguity
//             && Math.abs(poseEst.get().estimatedPose.getZ())
//                 < maxZError // and it has a realistic Z coordinate
//             && poseEst.get().estimatedPose.getX() > 0.0 // and it's within the field boundaries
//             && poseEst.get().estimatedPose.getX() < aprilTagLayout.getFieldLength()
//             && poseEst.get().estimatedPose.getY() > 0.0
//             && poseEst.get().estimatedPose.getY()
//                 < aprilTagLayout.getFieldWidth()) { // then accept the pose
//           consumer.accept(poseEst.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
//         }
//       }
//     }
//   }

//   private void updateRightCam() {
//     if (photonCameraRight.isConnected()) {
//       for (var result : photonCameraRight.getAllUnreadResults()) {
//         var poseEst =
//             result.getMultiTagResult().isPresent()
//                 ? rightEstimator.estimateCoprocMultiTagPose(result)
//                 : rightEstimator.estimateLowestAmbiguityPose(result);
//         if (poseEst.isPresent() // if there IS an estimate
//             && (result.multitagResult.isPresent()
//                 || Arrays.binarySearch(bannedTags, result.targets.get(0).fiducialId)
//                     < 0) // and it's either multitag or not banned
//             && result.getBestTarget().poseAmbiguity < maxAmbiguity // and low ambiguity
//             && Math.abs(poseEst.get().estimatedPose.getZ())
//                 < maxZError // and it has a realistic Z coordinate
//             && poseEst.get().estimatedPose.getX() > 0.0 // and it's within the field boundaries
//             && poseEst.get().estimatedPose.getX() < aprilTagLayout.getFieldLength()
//             && poseEst.get().estimatedPose.getY() > 0.0
//             && poseEst.get().estimatedPose.getY()
//                 < aprilTagLayout.getFieldWidth()) { // then accept the pose
//           consumer.accept(poseEst.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
//         }
//       }
//     }
//   }

//   public void setPDH(boolean enabled) {
//     powerModuleIO.setPDH(enabled);
//   }

//   public Command setPDHCommand(boolean enabled) {
//     return new InstantCommand(
//         () -> {
//           setPDH(enabled);
//         });
//   }

//   @FunctionalInterface
//   public static interface VisionConsumer {
//     public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds);
//   }
// }
