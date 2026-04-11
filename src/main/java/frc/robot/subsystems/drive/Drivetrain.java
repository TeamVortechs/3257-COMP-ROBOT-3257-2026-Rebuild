package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.VortechsUtil;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Drivetrain extends SubsystemBase {



  private PhotonCamera m_LeftCamera;
  private PhotonCamera m_RightCamera;
  private PhotonPoseEstimator m_LeftCameraEstimator;
  private PhotonPoseEstimator m_RightCameraEstimator;

  AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

  private double allianceMultipler = 1;

  private DrivetrainIO drivetrainIO;

  private DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

  public Drivetrain(DrivetrainIO drivetrainIO) {

    this.drivetrainIO = drivetrainIO;

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::runVelocityPathplanner,
        DriveConstants.PATHPLANNER_CONTROLLER,
        DriveConstants.PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    ConfigureCameras();
  }

  private void ConfigureCameras() {
    this.m_LeftCamera = new PhotonCamera(VisionConstants.photonLeftName);
    this.m_LeftCameraEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.robotToPhotonLeft);
    this.m_RightCamera = new PhotonCamera(VisionConstants.photonRightName);
    this.m_RightCameraEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.robotToPhotonRight);
  }

  private void UpdateForwardCamera() {
    if (m_LeftCamera.isConnected()) {
      List<PhotonPipelineResult> list = m_LeftCamera.getAllUnreadResults();
      if (!list.isEmpty()) {
        Optional<EstimatedRobotPose> estimatedPose = m_LeftCameraEstimator.update(list.get(0));
        if (estimatedPose.isPresent()) {
          this.addVisionMeasurement(
              estimatedPose.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
        }
      }
    }
  }

  private void UpdateFarCamera() {
    if (m_RightCamera.isConnected()) {
      List<PhotonPipelineResult> list = m_RightCamera.getAllUnreadResults();
      if (!list.isEmpty()) {
        Optional<EstimatedRobotPose> estimatedPose = m_RightCameraEstimator.update(list.get(0));
        if (estimatedPose.isPresent()) {
          this.addVisionMeasurement(
              estimatedPose.get().estimatedPose.toPose2d(), Utils.getCurrentTimeSeconds());
        }
      }
    }
  }

  @Override
  public void periodic() {
    drivetrainIO.updateInputs(inputs);
    Logger.processInputs("drivetrain", inputs);

    UpdateFarCamera();
    UpdateForwardCamera();
  }

  public void runVelocity(ChassisSpeeds speeds, boolean openLoop) {
    drivetrainIO.runRobotCentricVelocity(speeds, openLoop);
  }

  public void runVelocityPathplanner(ChassisSpeeds speeds) {
    runVelocity(speeds, false);
  }

  public void updateAllianceMultiplier() {
    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == Alliance.Blue) {
      allianceMultipler = 1;
    } else {
      allianceMultipler = -1;
    }
  }

  // drive commands
  public Command runVelocityCommand(ChassisSpeeds speeds, boolean openLoop) {

    return Commands.run(
        () -> {
          runVelocity(speeds, openLoop);
        },
        this);
  }

  public Command joystickDrive(CommandXboxController controller) {
    return joystickDrive(
        () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX());
  }

  public Command joystickDriveAtTarget(CommandXboxController controller) {
    return joystickDriveAtTarget(() -> -controller.getLeftY(), () -> -controller.getLeftX());
  }

  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double xSpeed =
              xSupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;
          double ySpeed =
              ySupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;

          xSpeed *= allianceMultipler;
          ySpeed *= allianceMultipler;

          double omegaSpeed =
              Math.copySign(
                  omegaSupplier.getAsDouble() * omegaSupplier.getAsDouble(),
                  omegaSupplier.getAsDouble());

          omegaSpeed *= DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC();

          ChassisSpeeds filteredSpeeds =
              DriveConstants.DRIVE_INPUT_FILTER.calculate(
                  new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));

          drivetrainIO.runFieldCentricVelocity(filteredSpeeds, DriveConstants.RUN_OPEN_LOOP_TELEOP);
        },
        this);
  }

  public Command joystickDriveRotation(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
    return Commands.run(
        () -> {
          double xSpeed =
              xSupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;
          double ySpeed =
              ySupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;

          xSpeed *= allianceMultipler;
          ySpeed *= allianceMultipler;

          ChassisSpeeds filteredSpeeds =
              DriveConstants.DRIVE_INPUT_FILTER.calculate(new ChassisSpeeds(xSpeed, ySpeed, 0));

          drivetrainIO.runFieldCentricVelocityAtRotation(filteredSpeeds, rotationSupplier.get(), DriveConstants.RUN_OPEN_LOOP_TELEOP);
        },
        this);
  }

  public Command joystickDriveAtTarget(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return joystickDriveRotation(
        xSupplier,
        ySupplier,
        () -> VortechsUtil.getHeadingToTarget(getPose(), rawTargetpose.get()));
  }

  public Command applyBrakeRequest() {
    return Commands.startRun(() -> drivetrainIO.runSwerveDriveBrake(), () -> {}, this);
  }

  public boolean isOriented() {

    boolean isOriented =
        Math.abs(
                getPose()
                    .getRotation()
                    .minus(VortechsUtil.getHeadingToTarget(getPose(), rawTargetpose.get()))
                    .getRadians())
            < DriveConstants.ORIENTATION_TOLERANCE;

    Logger.recordOutput("Drivetrain/isOriented", isOriented);

    return isOriented;
  }

  public double getDistanceToTarget() {

    double distanceToTarget =
        getPose().getTranslation().getDistance(rawTargetpose.get().getTranslation());

    Logger.recordOutput("Drivetrain/distanceToTarget", distanceToTarget);

    return distanceToTarget;
  }

  // pose related commands
  public void resetPose(Pose2d pose) {
    drivetrainIO.setPose(pose);
  }

  public Pose2d getPose() {
    return drivetrainIO.getPose();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return drivetrainIO.getChassisSpeeds();
  }

  // vision

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    drivetrainIO.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
  /** Adds a new timestamped vision measurement with no given standard deviations. */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    drivetrainIO.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public boolean isRightSideZone() {

    boolean isRightSideZone = VortechsUtil.isWithinYZone(4.05, false, getPose());
    ;

    Logger.recordOutput("Drivetrain/isRightSideZone", isRightSideZone);

    return isRightSideZone;
  }

  // pose shot stuff
  /**
   * The supplier for where the drivetrain is currrently aiming. Does not account for shooting on
   * the move
   */
  public Supplier<Pose2d> rawTargetpose =
      () -> {
        if (isInScoringZone()) {
          // in this case ur shooting

          Logger.recordOutput("Drivetrain/TargetPose", DriveConstants.GOAL_POSE.get());
          return DriveConstants.GOAL_POSE.get();
        }

        if (getPose().getY() < DriveConstants.CENTER_POINT.getY()) {
          Logger.recordOutput("Drivetrain/TargetPose", DriveConstants.PASSING_POSE_DOWN.get());
          return DriveConstants.PASSING_POSE_DOWN.get();
        }

        Logger.recordOutput("Drivetrain/TargetPose", DriveConstants.PASSING_POSE_UP.get());
        return DriveConstants.PASSING_POSE_UP.get();
      };

  public boolean isInScoringZone() {

    boolean isInScoringZone =
        VortechsUtil.isWithinXZone(DriveConstants.X_POSE_TO_PASS, false, getPose());
    Logger.recordOutput("Drivetrain/isInScoringZone", isInScoringZone);

    return isInScoringZone;
  }

  public Command sysIdQuasistatic(Direction direction) {
    // done with instant command for requirements
    return new InstantCommand(() -> {}, this)
        .andThen(drivetrainIO.sysIdQuasistaticCommand(direction));
  }

  public Command sysIdDynamic(Direction direction) {
    // done with instant command for requirements
    return new InstantCommand(() -> {}, this).andThen(drivetrainIO.sysIdDynamicCommand(direction));
  }

  // helper methods

}
