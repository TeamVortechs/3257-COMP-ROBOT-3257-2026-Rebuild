package frc.robot.subsystems.vision.detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.Random;

// simulation class just for testing purposes
public class DetectionIOSimulation implements DetectionIO {

  private Pose2d objectPose;
  private Timer timer;
  private double timeToSwitchSec = 60;

  private boolean gotPose = false;

  private Drive drive;

  private Random random;

  private boolean isDetected = true;

  public DetectionIOSimulation(Drive drive) {
    this.drive = drive;

    timer = new Timer();
    timer.start();

    objectPose = drive.getPose();

    random = new Random();
  }

  public void updateInputs(DetectionIOInputsAutoLogged inputs) {
    inputs.closestDetectionPose = objectPose;
    inputs.isDetected = isDetected;
    inputs.numberOfObjects = 1;
    inputs.objectDistance =
        objectPose.getTranslation().getDistance(drive.getPose().getTranslation());
  }

  public void update() {
    // if (timer.hasElapsed(timeToSwitchSec)) {
    //   timer.restart();
    //   objectPose = new Pose2d(drive.getPose().getX(), drive.getPose().getY(),
    // drive.getRotation());
    //   // new Pose2d(
    //   //     drive.getPose().getX() + random.nextDouble(-2, 2),
    //   //     drive.getPose().getY() + random.nextDouble(-2, 2),
    //   //     drive.getRotation());
    // }
  }

  public Pose2d getObjectPosition() {

    if (gotPose == false) {
      objectPose = new Pose2d(drive.getPose().getX(), drive.getPose().getY(), drive.getRotation());
      new Pose2d(drive.getPose().getX(), drive.getPose().getY(), drive.getRotation());

      gotPose = true;
    }

    return objectPose;
  }

  public Rotation2d getHeading() {

    if (!isDetected()) return new Rotation2d();

    Translation2d delta = objectPose.getTranslation().minus(drive.getPose().getTranslation());

    return new Rotation2d(delta.getX(), delta.getY());
  }

  public boolean isDetected() {
    return isDetected;
  }
}
