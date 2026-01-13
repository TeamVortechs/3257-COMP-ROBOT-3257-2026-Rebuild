// package frc.robot.subsystems.vision.detection;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.subsystems.drive.Drive;
// import java.util.List;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class DetectionIOPhotonVision implements DetectionIO {

//   // the camera
//   private PhotonCamera camera;

//   // drive system
//   private Drive drive;

//   // the id of the object that is tracking
//   private int objectID;

//   // the cloest field object
//   private FieldObject closestDetection = null;

//   // outputs
//   private Pose2d closestDetectionPose = null;
//   private boolean isDetected = false;

//   public DetectionIOPhotonVision(String name, Drive drive, int objectID) {
//     camera = new PhotonCamera(name);

//     this.drive = drive;
//     this.objectID = objectID;
//   }

//   @Override
//   public void updateInputs(DetectionIOInputsAutoLogged inputsAutoLogged) {
//     inputsAutoLogged.closestDetectionPose = closestDetectionPose;
//     inputsAutoLogged.isDetected = isDetected;

//     inputsAutoLogged.numberOfObjects = 0;

//     if (!isDetected) {
//       inputsAutoLogged.objectDistance = -1;
//     }
//     inputsAutoLogged.objectDistance =
//         closestDetectionPose.getTranslation().getDistance(drive.getPose().getTranslation());
//   }

//   // interface methods

//   @Override
//   public void update() {

//     double curTime = Timer.getFPGATimestamp();

//     List<PhotonPipelineResult> results = camera.getAllUnreadResults();

//     if (results.isEmpty()) {
//       isDetected = false;
//       return;
//     }

//     PhotonPipelineResult result = results.get(0);

//     if (!result.hasTargets()) {
//       isDetected = false;
//       return;
//     }

//     isDetected = true;

//     PhotonTrackedTarget pTarget = result.getBestTarget();
//     Transform3d offset = pTarget.getBestCameraToTarget();

//     Translation2d pose = new Translation2d(offset.getMeasureX(), offset.getMeasureY());

//     FieldObject obj = new FieldObject(pose, curTime);

//     closestDetection = obj;
//   }

//   /**
//    * gets the rotation that the robot needs to be to pick this up. PERHAPS THIS NEEDS TX TO WORK
//    * WELL. EXPERIMENT
//    */
//   @Override
//   public Rotation2d getHeading() {

//     if (!isDetected()) return new Rotation2d();

//     Translation2d delta = closestDetection.position.minus(drive.getPose().getTranslation());

//     return new Rotation2d(delta.getX(), delta.getY());
//   }

//   /**
//    * Gets the position of the object. Use isDetected first
//    *
//    * <p>If no object is detected it will return the position of the robot
//    */
//   @Override
//   public Pose2d getObjectPosition() {
//     return closestDetectionPose;
//   }

//   /**
//    * Gets wether or not the robot detects the object. This should always be used before get
// object
//    * position
//    */
//   @Override
//   public boolean isDetected() {
//     return isDetected;
//   }

//   // how we define an object that we are detecting. Data holder
//   public static class FieldObject {
//     Translation2d position;
//     double detectionTime;

//     public FieldObject(Translation2d position, double detectionTime) {
//       this.position = position;
//       this.detectionTime = detectionTime;
//     }
//   }
// }
