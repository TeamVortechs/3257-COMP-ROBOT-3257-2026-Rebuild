package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends CommandSwerveDrivetrain {

  private ShooterRotationManager shooterRotationManager;
  private GoalPoseManager goalPoseManager;

  private BuiltInAccelerometer accelerometer;

  private SwerveRequest.RobotCentric m_RobotCentricReq;
  private SwerveRequest.FieldCentric m_FieldCentricReq = new SwerveRequest.FieldCentric();

  private Supplier<Rotation2d> rotationSupplier =
      () -> {
        if (isWithinPassingZone()) {
          return getHeadingToPassing();
        } else {
          return getHeadingToGoal();
        }
      };

  public Drivetrain(
      SwerveDrivetrainConstants swerveDrivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(swerveDrivetrainConstants, modules);

    this.configureDrivetrain();
  }

  private void configureDrivetrain() {
    m_RobotCentricReq = new SwerveRequest.RobotCentric();
  }

  /*
   * Code I added(Ben). Putting this here so it's easier to know when we change the drive later
   */

  // puttign this in container because it's already like this and when we change drive it'll be good
  // to have it easily changed. Also it's probably more readable this way

  // caching values here so they can be logged
  // @AutoLogOutput private double accelerometerX;
  // @AutoLogOutput private double accelerometerY;

  private double accelerometerX;
  private double accelerometerY;

  /**
   * @return the needed rotation for the robot to rotate towards a goal I made it like this so we
   *     can use the joystick drive command from drive commands compensates for robot movement
   */
  public Rotation2d getHeadingToGoal() {
    goalPoseManager.setIsPassing(false);
    return shooterRotationManager.getHeading();
  }

  public Command logDistance() {
    return new RunCommand(
        () -> {
          Logger.recordOutput(
              "ShootingOnMove/Distance",
              getState().Pose.getTranslation().getDistance(new Translation2d(4.629, 4.024)));
        });
  }

  public Rotation2d getHeadingToPassing() {
    goalPoseManager.setIsPassing(true);

    if (getState().Pose.getY() > DriveConstants.HALF_MAP_Y) {
      // setPassingIndexCommmand(0).schedule();
      // this iss the
      goalPoseManager.setPassingPoseIndex(0);
    } else {
      goalPoseManager.setPassingPoseIndex(1);

      // setPassingIndexCommmand(1).schedule();
    }

    return shooterRotationManager.getHeading();
  }

  /**
   * @return wether or not the robot is in a zone where the shooter hsould be charged more
   *     agressively to reduce windup time
   */
  @AutoLogOutput
  public boolean isWithinShooterAutomaticChargingZone() {
    return isWithinZone(DriveConstants.X_POSE_TO_CHARGE, false);
  }

  @AutoLogOutput
  public boolean isWithinPassingZone() {
    return isWithinZone(DriveConstants.X_POSE_TO_PASS, true);
  }

  public Rotation2d getRotationOverBumper() {
    double xPose = getState().Pose.getX();

    if (xPose > Constants.DriveConstants.CENTER_POINT.getX()) {
      // this means we're on the red side
      return Rotation2d.fromDegrees(Constants.DriveConstants.RED_SIDE_DEGREES);
    } else {
      // this means we're on the blue side
      return Rotation2d.fromDegrees(Constants.DriveConstants.BLUE_SIDE_DEGREES);
    }
  }

  private boolean isWithinZone(double x, boolean wantsCenter) {
    double xPose = getState().Pose.getX();

    if (DriverStation.getAlliance().isEmpty()) {
      return false;
    }

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      if (!wantsCenter) {
        return xPose < x;
      } else {
        return xPose > x;
      }
    } else {
      if (!wantsCenter) {
        return xPose > (2 * DriveConstants.CENTER_POINT.getX() - x);
      } else {
        return xPose < (2 * DriveConstants.CENTER_POINT.getX() - x);
      }
    }
  }

  /**
   * @return the distance from the robot to the goal
   */
  public double getDistanceToGoal() {

    // makes the goal pose manager switch to the shooter setting
    goalPoseManager.setIsPassing(false);
    return shooterRotationManager.getDistance();
  }

  public double getDistanceToPassing() {
    ///
    goalPoseManager.setIsPassing(true);
    return shooterRotationManager.getDistance();
  }

  public double getDistanceToTarget() {
    return shooterRotationManager.getDistance();
  }

  /**
   * @return wether or not the shooter is pointing towards the goal within tolerance
   */
  public boolean isPointingToGoal() {
    return shooterRotationManager.isOriented();
  }

  public void setShootingOnMove(boolean calculateShootingOnMove) {
    shooterRotationManager.setCalculateShootMove(calculateShootingOnMove);
  }

  // commands
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
          double xSpeed = xSupplier.getAsDouble() * getMaxLinearSpeedMetersPerSec();
          double ySpeed = ySupplier.getAsDouble() * getMaxLinearSpeedMetersPerSec();

          double omegaSpeed =
              Math.copySign(
                  omegaSupplier.getAsDouble() * omegaSupplier.getAsDouble(),
                  omegaSupplier.getAsDouble());

          omegaSpeed *= getMaxAngularSpeedRadPerSec();

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

          double xSpeed = xSupplier.getAsDouble() * getMaxLinearSpeedMetersPerSec();
          double ySpeed = ySupplier.getAsDouble() * getMaxLinearSpeedMetersPerSec();

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
    return joystickDriveRotation(xSupplier, ySupplier, rotationSupplier);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_BASE_RADIUS;
  }
}
