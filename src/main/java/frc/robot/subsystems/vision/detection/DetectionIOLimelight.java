package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.LimelightHelpers.*;
import java.util.ArrayList;
import java.util.List;

public class DetectionIOLimelight implements DetectionIO {

  // name of the limelight
  private String name;

  // drive system
  private Drive drive;

  // the id of the object that is tracking
  private int objectID;

  // the list of field objects currently being tracked
  private List<FieldObject> fieldObjects;

  // the cloest field object
  private FieldObject closestDetection = null;

  // outputs
  private Pose2d closestDetectionPose = null;
  private boolean isDetected = false;

  public DetectionIOLimelight(String name, Drive drive, int objectID) {
    this.name = name;
    this.drive = drive;
    this.objectID = objectID;

    fieldObjects = new ArrayList<>();
  }

  @Override
  public void updateInputs(DetectionIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.closestDetectionPose = closestDetectionPose;
    inputsAutoLogged.isDetected = isDetected;

    inputsAutoLogged.numberOfObjects = fieldObjects.size();

    if (!isDetected) {
      inputsAutoLogged.objectDistance = -1;
    }
    inputsAutoLogged.objectDistance =
        closestDetectionPose.getTranslation().getDistance(drive.getPose().getTranslation());
  }

  // interface methods

  @Override
  public void update() {

    double curTime = Timer.getFPGATimestamp();

    // remove old objects from the list if they were added to long ago
    fieldObjects.removeIf(
        (object) -> curTime - object.detectionTime > VisionConstants.objTimeoutTimeSec);

    // thin the list to 20 of the most recent detections. Slightly inneficient(n^2 theory). Fine for
    // 20 objects
    while (fieldObjects.size() > VisionConstants.maxObjAmount) {
      fieldObjects.remove(0);
    }

    // get the filtered list of new objects
    List<RawDetection> detections = getFilteredRawDetections();
    List<FieldObject> newFieldObjects = detectionsToFieldObjects(detections, curTime);

    // updates the old list of objects with the new one
    fieldObjects.addAll(newFieldObjects);

    closestDetection = getClosestFieldObject(drive.getPose(), fieldObjects);

    // sets wether it detected something to wether a detection exists
    isDetected = closestDetection != null;

    if (isDetected) {
      closestDetectionPose = new Pose2d(closestDetection.position, new Rotation2d());
    } else {
      // reduncy to make sure the robot never runs across the stage or anything
      closestDetectionPose = drive.getPose();
    }
  }

  /**
   * gets the rotation that the robot needs to be to pick this up. PERHAPS THIS NEEDS TX TO WORK
   * WELL. EXPERIMENT
   */
  @Override
  public Rotation2d getHeading() {

    if (!isDetected()) return new Rotation2d();

    Translation2d delta = closestDetection.position.minus(drive.getPose().getTranslation());

    return new Rotation2d(delta.getX(), delta.getY());
  }

  /**
   * Gets the position of the object. Use isDetected first
   *
   * <p>If no object is detected it will return the position of the robot
   */
  @Override
  public Pose2d getObjectPosition() {
    return closestDetectionPose;
  }

  /**
   * Gets wether or not the robot detects the object. This should always be used before get object
   * position
   */
  @Override
  public boolean isDetected() {
    return isDetected;
  }

  // logic methods

  /** get a list of raw detections that has been filtered out according to our specifications */
  public List<RawDetection> getFilteredRawDetections() {
    RawDetection[] detections = LimelightHelpers.getRawDetections(name);

    // System.out.println("raw detections detection IO limelight");
    for (int i = 0; i < detections.length; i++) {
      System.out.println(detections[i].classId);
    }

    List<RawDetection> filteredDetections = new ArrayList<>();

    // null check
    if (detections == null) return new ArrayList<>();

    // loop through every detection for validiation
    for (int i = 0; i < detections.length; i++) {
      RawDetection currDetection = detections[i];

      // make sure it is the correct object
      if (currDetection.classId != objectID) continue;

      // at this point it has passed all required checks and is ready to add to the complete list
      filteredDetections.add(currDetection);
    }

    return filteredDetections;
  }

  /**
   * converts raw detections to field relative poses.
   *
   * @param detections
   * @return
   */
  public List<FieldObject> detectionsToFieldObjects(
      List<RawDetection> detections, double timestamp) {

    List<FieldObject> output = new ArrayList<>();

    // System.out.println("detections to field obkjects");

    // go through every detection and convert it to a field-relative pose
    for (RawDetection detection : detections) {
      double tx = detection.txnc;
      double ty = detection.tync;

      // calculates the robot-relative offset given the tx and ty values
      Translation2d offset = calcDistToObject(tx, ty);
      Pose2d fieldPose =
          drive.convertFieldRelative(new Transform2d(offset, Rotation2d.fromDegrees(0)));

      output.add(new FieldObject(fieldPose.getTranslation(), timestamp));
    }

    return output;
  }

  /**
   * returns the closest field object to the given pose
   *
   * @param pose
   * @param objects
   * @return
   */
  public FieldObject getClosestFieldObject(Pose2d pose, List<FieldObject> objects) {

    if (objects.isEmpty()) return null;

    FieldObject closest = objects.get(0);

    for (FieldObject fieldObject : objects) {

      if (fieldObject.position.getDistance(pose.getTranslation())
          < closest.position.getDistance(pose.getTranslation())) {
        closest = fieldObject;
      }
    }

    return closest;
  }

  // how we define an object that we are detecting. Data holder
  public static class FieldObject {
    Translation2d position;
    double detectionTime;

    public FieldObject(Translation2d position, double detectionTime) {
      this.position = position;
      this.detectionTime = detectionTime;
    }
  }

  // helper methods
  /** tx: horizantonal offset in degrees (CW+) of object from camera ty: vertical offset (CW+) */

  // LIFTED DIRECTLY FROM CITRUS
  // we aim at the lower end of the object. This works for coral but maybe not for algae. Test and
  // report back. Also keep in mind we can use TA and confidence from json if needed
  public Translation2d calcDistToObject(double tx, double ty) {

    Transform3d cameraOffset = VisionConstants.ROBOT_TO_ARDUCAM_DETECTION;
    Distance algaeRad = VisionConstants.ALGAE_RADIUS;

    // verticlal angle
    double totalAngleY = Units.degreesToRadians(-ty) - cameraOffset.getRotation().getY();

    // System.out.println("total angle: " + totalAngleY);

    // all this is doing is : horizantol = z/tan(angle)
    Distance distAwayY =
        // the hieght of the camera
        cameraOffset
            .getMeasureZ() // get camera height
            .minus(algaeRad) // find camera-to-algae-middle height
            .div(Math.tan(totalAngleY)); // outputs "X" robot-relative coordinate
    // System.out.println("dist away y" + distAwayY);

    // hypotenuse of triangle formed by height of robot to center of algae and the x distance
    Distance distHypotenuseYToGround =
        BaseUnits.DistanceUnit.of(
            Math.hypot(
                distAwayY.in(BaseUnits.DistanceUnit),
                cameraOffset.getMeasureZ().minus(algaeRad).in(BaseUnits.DistanceUnit)));

    // System.out.println("dist hypo to ground: " + distHypotenuseYToGround);

    // same thing as before
    double totalAngleX = Units.degreesToRadians(-tx) + cameraOffset.getRotation().getZ();

    // System.out.println("total angle x: " + totalAngleX);

    Distance distAwayX =
        distAwayY.times(Math.tan(totalAngleX)); // outputs "Y" robot-relative coordinate

    // System.out.println("dist away x: " + distAwayX);

    SmartDashboard.putNumber(name + "/tx", tx);
    SmartDashboard.putNumber(name + "/ty", ty);
    SmartDashboard.putNumber(
        name + "/Distance Away Y", distAwayY.in(edu.wpi.first.units.Units.Meters));
    SmartDashboard.putNumber(
        name + "/Distance Away X", distAwayX.in(edu.wpi.first.units.Units.Meters));
    SmartDashboard.putNumber(
        name + "/Distance Away Hyp ", distHypotenuseYToGround.in(edu.wpi.first.units.Units.Meters));

    // return new Translation2d(distAwayY, distAwayX);
    return new Translation2d(
        distAwayX, distAwayY.times(-1)); // we rotated the entire thing by 90deg clockwise
  }
}
