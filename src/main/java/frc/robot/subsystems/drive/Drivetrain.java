package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.VortechsUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Drivetrain extends CommandSwerveDrivetrain {

  private SwerveRequest.RobotCentric m_RobotCentricReq = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric m_FieldCentricReq = new SwerveRequest.FieldCentric();

  private ShootOnMoveManager shootOnMoveManager;

  public Drivetrain(
      SwerveDrivetrainConstants swerveDrivetrainConstants, 
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(swerveDrivetrainConstants, modules);

    // necessary prerequisites to configuring autobuilder
    PPHolonomicDriveController pathplannerController =
        new PPHolonomicDriveController(
            new PIDConstants(
                DriveConstants.TRANS_KP, DriveConstants.TRANS_KI, DriveConstants.TRANS_KD),
            new PIDConstants(
                DriveConstants.ANGLE_KP, DriveConstants.ANGLE_KI, DriveConstants.ANGLE_KD));
    RobotConfig PP_CONFIG =
        new RobotConfig(
            DriveConstants.ROBOT_WEIGHT,
            DriveConstants.ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                DriveConstants.WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            getModuleLocations());

    // set update rates for drivetrain status signals
    for (var module : getModules()) {
      var driveMotor = module.getDriveMotor();
      var steerMotor = module.getSteerMotor();
      BaseStatusSignal.setUpdateFrequencyForAll(
          DriveConstants.FREQUENCY_DRIVETRAIN,
          driveMotor.getPosition(),
          driveMotor.getVelocity(),
          driveMotor.getVelocity());
      BaseStatusSignal.setUpdateFrequencyForAll(
          DriveConstants.FREQUENCY_DRIVETRAIN,
          steerMotor.getPosition(),
          steerMotor.getVelocity(),
          steerMotor.getVelocity());
      driveMotor.optimizeBusUtilization(DriveConstants.FREQUENCY_DRIVETRAIN);
      steerMotor.optimizeBusUtilization(DriveConstants.FREQUENCY_DRIVETRAIN);
    }
    // also set the pigeon signal
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.FREQUENCY_DRIVETRAIN, getPigeon2().getAngularVelocityZWorld());
    getPigeon2().optimizeBusUtilization(DriveConstants.FREQUENCY_DRIVETRAIN);

    // shoot on move manager here. likely to be removed in the future
    shootOnMoveManager = new ShootOnMoveManager(rawTargetpose, this);

    // configure autobuilder for pathplanner
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::runVelocity,
        pathplannerController,
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
      Pathfinding.setPathfinder(new LocalADStarAK());
  }

  // periodic functions to help with logging
  @Override
  public void periodic() {
    updateTelemetry(); // not to be confused with this.registerTelemetry()
  }

  private void updateTelemetry() {
        // update our status signals
    for (var module : getModules()) {
      var driveMotor = module.getDriveMotor();
      var steerMotor = module.getSteerMotor();
      BaseStatusSignal.refreshAll(
          driveMotor.getPosition(), driveMotor.getVelocity(), driveMotor.getVelocity());
      BaseStatusSignal.refreshAll(
          steerMotor.getPosition(), steerMotor.getVelocity(), steerMotor.getVelocity());
    }
    BaseStatusSignal.refreshAll(getPigeon2().getAngularVelocityZWorld());

    System.out.println("current read pose: " + getPose().toString());
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

  @AutoLogOutput
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
  //   private SwerveModulePosition[] lastModulePositions = // For delta tracking
  //     new SwerveModulePosition[] {
  //       new SwerveModulePosition(),
  //       new SwerveModulePosition(),
  //       new SwerveModulePosition(),
  //       new SwerveModulePosition()
  //     };
  // SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(getKinematics(),
  // Rotation2d.kZero, lastModulePositions, getPose());

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
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
