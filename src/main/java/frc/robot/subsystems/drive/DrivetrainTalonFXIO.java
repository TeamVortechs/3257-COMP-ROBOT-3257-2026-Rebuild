package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

public class DrivetrainTalonFXIO extends CommandSwerveDrivetrain implements DrivetrainIO {
  private SwerveRequest.ApplyFieldSpeeds m_ApplyFieldSpeeds;
  private SwerveRequest.ApplyRobotSpeeds m_ApplyRobotSpeeds;
  private SwerveRequest.FieldCentric m_FieldCentric;
  private SwerveRequest.FieldCentricFacingAngle m_FieldCentricFacingAngle;
  private SwerveRequest.Idle m_Idle;
  private SwerveRequest.PointWheelsAt m_PointWheelsAt;
  private SwerveRequest.RobotCentric m_RobotCentric;
  private SwerveRequest.RobotCentricFacingAngle m_RobotCentricFacingAngle;
  private SwerveRequest.SwerveDriveBrake m_Brake;

  private SwerveRequest.SwerveDriveBrake m_SwerveDriveBrake = new SwerveRequest.SwerveDriveBrake();
  private SwerveRequest.RobotCentric m_RobotCentricReq = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric m_FieldCentricReq = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle m_FieldCentricAngleReq =
      new SwerveRequest.FieldCentricFacingAngle()
          .withHeadingPID(
              DriveConstants.ANGLE_KP, DriveConstants.ANGLE_KI, DriveConstants.ANGLE_KD);
  private StatusSignal<LinearAcceleration> m_XAccelSignal;
  private StatusSignal<LinearAcceleration> m_YAccelSignal;
  private SwerveDriveState m_CurrentStateCopy;

  public DrivetrainTalonFXIO(SwerveDrivetrainConstants swerveDrivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(swerveDrivetrainConstants, modules);
    m_XAccelSignal = this.getPigeon2().getAccelerationX();
    m_YAccelSignal = this.getPigeon2().getAccelerationY();
    m_CurrentStateCopy = this.getStateCopy();
    this.initializeSwerveRequests();
  }

  public void updateInputs(DrivetrainIOInputsAutoLogged inputsAutoLogged) {
    BaseStatusSignal.refreshAll(m_XAccelSignal, m_YAccelSignal);
    m_CurrentStateCopy = this.getStateCopy();
    inputsAutoLogged.chassisSpeeds = this.getChassisSpeeds();
    inputsAutoLogged.pose = this.getPose();
    inputsAutoLogged.xAcceleration = this.getXAcceleration();
    inputsAutoLogged.yAcceleration = this.getYAcceleration();
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

  public void runSwerveDriveBrake() {
    setControl(m_SwerveDriveBrake);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.m_CurrentStateCopy.Speeds;
  }

  public Pose2d getPose() {
    return this.m_CurrentStateCopy.Pose;
  }

  /**
   * gets input from accelerometer
   * assumes that the status signal has already been refreshed before calling
   * @return
   */
  public double getXAcceleration() {
    return this.m_XAccelSignal.getValueAsDouble();
  }

  /**
   * gets input from accelerometer
   * assumes that the status signal has already been refreshed before calling
   * @return
   */
  public double getYAcceleration() {
    return this.m_YAccelSignal.getValueAsDouble();
  }

  public void setPose(Pose2d pose) {
    resetPose(pose);
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return sysIdQuasistatic(direction);
  }

  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return sysIdDynamic(direction);
  }

  private void initializeSwerveRequests() {
    m_ApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
                                          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    m_ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
                                          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    m_FieldCentric = new SwerveRequest.FieldCentric()
                                      .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    m_FieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                                                 .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    m_Idle = new SwerveRequest.Idle();
    m_PointWheelsAt = new SwerveRequest.PointWheelsAt();
    m_RobotCentric = new SwerveRequest.RobotCentric();
    m_RobotCentricFacingAngle = new SwerveRequest.RobotCentricFacingAngle();
    m_Brake = new SwerveRequest.SwerveDriveBrake();
  }
}
