package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Detector extends SubsystemBase {
  private DetectionIO detectionIO;
  private DetectionIOInputsAutoLogged inputs = new DetectionIOInputsAutoLogged();

  public Detector(DetectionIO detectionIO) {
    this.detectionIO = detectionIO;
  }

  @Override
  public void periodic() {
    detectionIO.update();

    detectionIO.updateInputs(inputs);
    Logger.processInputs("detection", inputs);
  }

  public boolean isDetected() {
    return detectionIO.isDetected();
  }

  public Pose2d getObjectPose() {

    Pose2d objectPose = detectionIO.getObjectPosition();

    // makes it so the robot will rotate towards where it is moving when driving to the pose
    Rotation2d heading = detectionIO.getHeading();

    return new Pose2d(objectPose.getTranslation(), heading);
  }
}
