package frc.robot.subsystems.drive.filtering;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DeadbandDriveInputFilter extends DriveInputFilter {

  private double transDeadband;
  private double rotDeadband;

  private double transMagnitude;
  private double rotMagnitude;

  public DeadbandDriveInputFilter(
      double transDeadband, double rotDeadband, double transMagnitude, double rotMagnitude) {
    this.transDeadband = transDeadband;
    this.rotDeadband = rotDeadband;
    this.transMagnitude = transMagnitude;
    this.rotMagnitude = rotMagnitude;
  }

  @Override
  protected ChassisSpeeds runFilter(ChassisSpeeds speeds) {
    speeds.vxMetersPerSecond =
        MathUtil.applyDeadband(speeds.vxMetersPerSecond, transDeadband, transMagnitude);
    speeds.vyMetersPerSecond =
        MathUtil.applyDeadband(speeds.vyMetersPerSecond, transDeadband, transMagnitude);
    speeds.omegaRadiansPerSecond =
        MathUtil.applyDeadband(speeds.omegaRadiansPerSecond, rotDeadband, rotMagnitude);

    return speeds;
  }
}
