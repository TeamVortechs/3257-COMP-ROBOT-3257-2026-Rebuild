package frc.robot.subsystems.drive.filtering;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DeadbandDriveInputFilter extends DriveInputFilter {

  private double transDeadband;
  private double rotDeadband;

  public DeadbandDriveInputFilter(double transDeadband, double rotDeadband) {
    this.transDeadband = transDeadband;
    this.rotDeadband = rotDeadband;
  }

  @Override
  protected ChassisSpeeds runFilter(ChassisSpeeds speeds) {
    speeds.vxMetersPerSecond = MathUtil.applyDeadband(speeds.vxMetersPerSecond, transDeadband);
    speeds.vyMetersPerSecond = MathUtil.applyDeadband(speeds.vyMetersPerSecond, transDeadband);
    speeds.omegaRadiansPerSecond =
        MathUtil.applyDeadband(speeds.omegaRadiansPerSecond, rotDeadband);

    return speeds;
  }
}
