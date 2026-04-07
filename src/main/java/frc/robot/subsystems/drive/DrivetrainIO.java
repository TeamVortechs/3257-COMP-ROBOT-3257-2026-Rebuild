package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {

  @AutoLog
  public static class DrivetrainIOInputs {
    double xSpeed;
    double ySpeed;
    double rotSpeed;
    Pose2d pose;
    double xAcceleration;
    double yAcceleration;
    Rotation2d heading;
  }

  public default void updateInputs(DrivetrainIOInputsAutoLogged inputsAutoLogged) {}

  public default void runRobotCentricVelocity(ChassisSpeeds chassisSpeeds) {}

  public default void runFieldCentricVelocity(ChassisSpeeds chassisSpeeds) {}

  public default void runFieldCentricVelocityAtRotation(
      ChassisSpeeds chassisSpeeds, Rotation2d rotation2d) {}

  public default void runSwerveDriveBrake() {}

  public default ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds();
  }

  /**
   * gets input from accelerometer
   *
   * @return
   */
  public default double getXAcceleration() {
    return 0;
  }

  /**
   * gets input from accelerometer
   *
   * @return
   */
  public default double getYAcceleration() {
    return 0;
  }

  public default void setPose(Pose2d pose) {}

  public default Pose2d getPose() {
    return new Pose2d();
  }

  public default Rotation2d getHeading() {
    return new Rotation2d();
  }

  public default Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return new InstantCommand();
  }

  public default Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return new InstantCommand();
  }

  public default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}
}
