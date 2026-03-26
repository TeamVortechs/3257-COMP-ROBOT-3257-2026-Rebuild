package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.VortechsUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Drivetrain extends CommandSwerveDrivetrain {

  private SwerveRequest.RobotCentric m_RobotCentricReq = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric m_FieldCentricReq = new SwerveRequest.FieldCentric();

  private ShootOnMoveManager shootOnMoveManager;

  public Drivetrain(
      SwerveDrivetrainConstants swerveDrivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(swerveDrivetrainConstants, modules);

    shootOnMoveManager = new ShootOnMoveManager(rawTargetpose, this);
  }

  // drive commands
  public Command runVelocity(ChassisSpeeds speeds) {

    return Commands.run(
        () -> {
          this.setControl(
              m_RobotCentricReq
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        },
        this);
  }

  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double xSpeed =
              xSupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;
          double ySpeed =
              ySupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;

          double omegaSpeed =
              Math.copySign(
                  omegaSupplier.getAsDouble() * omegaSupplier.getAsDouble(),
                  omegaSupplier.getAsDouble());

          omegaSpeed *= DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC();

          this.setControl(
              m_FieldCentricReq
                  .withVelocityX(xSpeed)
                  .withVelocityY(ySpeed)
                  .withRotationalRate(omegaSpeed));
        },
        this);
  }

  public Command joystickDriveRotation(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
    return Commands.run(
        () -> {
          ProfiledPIDController angleController = DriveConstants.ANGLE_CONTROLLER;

          double xSpeed =
              xSupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;
          double ySpeed =
              ySupplier.getAsDouble() * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND;

          double omegaSpeed =
              angleController.calculate(
                  getState().Pose.getRotation().getRadians(), rotationSupplier.get().getRadians());

          this.setControl(
              m_FieldCentricReq
                  .withVelocityX(xSpeed)
                  .withVelocityX(ySpeed)
                  .withRotationalRate(omegaSpeed)
                  .withDeadband(DriveConstants.ANGLE_DEADBAND));
        },
        this);
  }

  public Command joystickDriveAtTarget(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    // turn on shoot on move, makes sure it is to save on calculations
    return new InstantCommand(
            () -> {
              shootOnMoveManager.setCalculateShootMove(true);
            })
        .andThen(joystickDriveRotation(xSupplier, ySupplier, () -> shootOnMoveManager.getHeading()))

        // turn off shoot on move calculations to save memory
        .andThen(
            new InstantCommand(
                () -> {
                  shootOnMoveManager.setCalculateShootMove(false);
                }));
  }

  public boolean isOriented() {
    return shootOnMoveManager.isOriented();
  }

  public double getDistanceToTarget() {
    return shootOnMoveManager.getDistance();
  }

  // pose related commands
  public void resetPose(Pose2d pose) {
    this.getState().Pose = pose;
  }

  public Pose2d getPose() {
    return this.getStateCopy().Pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.getStateCopy().Speeds;
  }

  // autonomous commands
  public Command overrideRotationCommand() {
    return new InstantCommand();
  }

  public Command removeRotationOverrideCommand() {
    return new InstantCommand();
  }

  // vision

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  // pose shot stuff
  /**
   * The supplier for where the drivetrain is currrently aiming. Does not account for shooting on
   * the move
   */
  public Supplier<Pose2d> rawTargetpose =
      () -> {
        if (VortechsUtil.isWithinXZone(DriveConstants.X_POSE_TO_PASS, false, getPose())) {
          // in this case ur shooting
          return DriveConstants.GOAL_POSE.get();
        }

        if (getPose().getY() < DriveConstants.CENTER_POINT.getY()) {
          return DriveConstants.PASSING_POSE_DOWN.get();
        }

        return DriveConstants.PASSING_POSE_UP.get();
      };

  // helper methods

}
