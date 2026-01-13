package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface DetectionIO {

  @AutoLog
  public static class DetectionIOInputs {
    Pose2d closestDetectionPose = null;
    boolean isDetected = false;
    int numberOfObjects = 0;
    double objectDistance = 0;
  }

  /** updates the inputs for advantage kit logging purposes */
  public default void updateInputs(DetectionIOInputsAutoLogged inputs) {}

  /** updates the vision systems. THis should be called in the periodic function of the subsystem */
  public default void update() {}
  ;

  /** gets the position of the object field relative */
  public default Pose2d getObjectPosition() {
    return new Pose2d(new Translation2d(), new Rotation2d());
  }

  /** Gets the rotation that the robot should be facing in order to pick up the object */
  public default Rotation2d getHeading() {
    return new Rotation2d();
  }

  /**
   * Gets wether or not the vision detects something. This should be called before getHeading or
   * getObjectPosition
   */
  public default boolean isDetected() {
    return false;
  }

  /** sets the pipeline of the limelight. This is currently not implemented */
  public default void setPipeline(int pipeline) {}
  ;
}
