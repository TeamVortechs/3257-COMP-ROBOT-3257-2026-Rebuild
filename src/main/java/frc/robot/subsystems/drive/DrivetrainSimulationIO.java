package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSimulationIO extends CommandSwerveDrivetrain implements DrivetrainIO {

  private SwerveRequest.RobotCentric m_RobotCentricReq =
      new SwerveRequest.RobotCentric()
          .withDeadband(0.1 * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND)
          .withRotationalDeadband(0.1 * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC());

  private SwerveRequest.FieldCentric m_FieldCentricReq =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.1 * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND)
          .withRotationalDeadband(0.1 * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC());

  private SwerveRequest.FieldCentricFacingAngle m_FieldCentricAngleReq =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(10, DriveConstants.ANGLE_KI, DriveConstants.ANGLE_KD)
          .withDeadband(0.1 * DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND)
          .withRotationalDeadband(0.1 * DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC());

  BuiltInAccelerometer builtInAccelerometer = new BuiltInAccelerometer();

  private double m_lastSimTime = 0;
  private Notifier m_simNotifier;

  public DrivetrainSimulationIO(
      SwerveDrivetrainConstants swerveDrivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(swerveDrivetrainConstants, modules);
  }

  public void updateInputs(DrivetrainIOInputsAutoLogged inputsAutoLogged) {

    inputsAutoLogged.xSpeed = getChassisSpeeds().vxMetersPerSecond;
    inputsAutoLogged.ySpeed = getChassisSpeeds().vyMetersPerSecond;
    inputsAutoLogged.rotSpeed = getChassisSpeeds().omegaRadiansPerSecond;

    inputsAutoLogged.pose = getPose();

    inputsAutoLogged.xAcceleration = getXAcceleration();
    inputsAutoLogged.yAcceleration = getYAcceleration();

    inputsAutoLogged.heading = getHeading();

    /* Use the measured time delta, get battery voltage from WPILib */
  }

  public void initialize() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    this.m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* Use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(0.000005);
  }

  public void runRobotCentricVelocity(ChassisSpeeds chassisSpeeds) {

    setControl(
        m_RobotCentricReq
            .withVelocityX(chassisSpeeds.vxMetersPerSecond)
            .withVelocityY(chassisSpeeds.vyMetersPerSecond)
            .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
  }

  public void runFieldCentricVelocity(ChassisSpeeds chassisSpeeds) {
    setControl(
        m_FieldCentricReq
            .withVelocityX(chassisSpeeds.vxMetersPerSecond)
            .withVelocityY(chassisSpeeds.vyMetersPerSecond)
            .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
  }

  public void runFieldCentricVelocityAtRotation(
      ChassisSpeeds chassisSpeeds, Rotation2d rotation2d) {
    setControl(
        m_FieldCentricAngleReq
            .withVelocityX(chassisSpeeds.vxMetersPerSecond)
            .withVelocityY(chassisSpeeds.vyMetersPerSecond)
            .withTargetDirection(rotation2d));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getStateCopy().Speeds;
  }

  /**
   * gets input from accelerometer
   *
   * @return
   */
  public double getXAcceleration() {
    return builtInAccelerometer.getX();
  }

  /**
   * gets input from accelerometer
   *
   * @return
   */
  public double getYAcceleration() {
    return builtInAccelerometer.getY();
  }

  public void setPose(Pose2d pose) {
    getState().Pose = pose;
  }

  public Pose2d getPose() {
    return getStateCopy().Pose;
  }

  public Rotation2d getHeading() {

    return getStateCopy().RawHeading;
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return sysIdQuasistatic(direction);
  }

  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return sysIdDynamic(direction);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    // super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
    // visionMeasurementStdDevs);
  }
}
