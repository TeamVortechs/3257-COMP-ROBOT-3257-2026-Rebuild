package frc.robot.subsystems.drive.filtering;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SlewDriveInputFilter extends DriveInputFilter {

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter angleLimiter;

  public SlewDriveInputFilter(double transLimiter, double angleLimiter) {
    xLimiter = new SlewRateLimiter(transLimiter);
    yLimiter = new SlewRateLimiter(transLimiter);
    this.angleLimiter = new SlewRateLimiter(angleLimiter);
  }

  @Override
  protected ChassisSpeeds runFilter(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        xLimiter.calculate(speeds.vxMetersPerSecond),
        yLimiter.calculate(speeds.vyMetersPerSecond),
        angleLimiter.calculate(speeds.omegaRadiansPerSecond));
  }
}
