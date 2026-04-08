package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.VortechsUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  private double allianceMultipler = 1;

  private DrivetrainIO drivetrainIO;

  private DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();

  public Drivetrain(DrivetrainIO drivetrainIO) {

    this.drivetrainIO = drivetrainIO;

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::runVelocity,
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
  }

  @Override
  public void periodic() {
    drivetrainIO.updateInputs(inputs);
    Logger.processInputs("drivetrain", inputs);

    Logger.recordOutput("drivetrain/targetPose", rawTargetpose.get());
  }

  public void runVelocity(ChassisSpeeds speeds) {
    drivetrainIO.runRobotCentricVelocity(speeds);
  }

  public void updateAllianceMultiplier() {
    if (Constants.ALLIANCE.get() == Alliance.Blue) {
      allianceMultipler = 1;
    } else {
      allianceMultipler = -1;
    }
  }

  // drive commands
  public Command runVelocityCommand(ChassisSpeeds speeds) {

    return Commands.run(
        () -> {
          drivetrainIO.runRobotCentricVelocity(speeds);
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

          drivetrainIO.runFieldCentricVelocity(filteredSpeeds);
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

          drivetrainIO.runFieldCentricVelocityAtRotation(filteredSpeeds, rotationSupplier.get());
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
    return getPose()
            .getRotation()
            .minus(VortechsUtil.getHeadingToTarget(getPose(), rawTargetpose.get()))
            .getRadians()
        < DriveConstants.ORIENTATION_TOLERANCE;
  }

  public double getDistanceToTarget() {
    return getPose().getTranslation().getDistance(rawTargetpose.get().getTranslation());
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

  @AutoLogOutput
  public boolean isRightSideZone() {
    return VortechsUtil.isWithinYZone(4.05, false, getPose());
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
          return DriveConstants.GOAL_POSE.get();
        }

        if (getPose().getY() < DriveConstants.CENTER_POINT.getY()) {
          return DriveConstants.PASSING_POSE_DOWN.get();
        }

        return DriveConstants.PASSING_POSE_UP.get();
      };

  public boolean isInScoringZone() {
    return (VortechsUtil.isWithinXZone(DriveConstants.X_POSE_TO_PASS, false, getPose()));
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
