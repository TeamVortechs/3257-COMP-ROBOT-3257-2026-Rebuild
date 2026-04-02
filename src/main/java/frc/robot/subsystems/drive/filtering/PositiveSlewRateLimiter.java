package frc.robot.subsystems.drive.filtering;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

// limits acceleration only when increasing
public class PositiveSlewRateLimiter extends DriveInputFilter {

  private PositiveSlewFilter xLimiter;
  private PositiveSlewFilter yLimiter;
  private PositiveSlewFilter omegaLimiter;

  public PositiveSlewRateLimiter(double transLimit, double omegaLimit) {
    xLimiter = new PositiveSlewFilter(transLimit);
    yLimiter = new PositiveSlewFilter(transLimit);
    omegaLimiter = new PositiveSlewFilter(omegaLimit);
  }

  @Override
  protected ChassisSpeeds runFilter(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        xLimiter.calculate(speeds.vxMetersPerSecond),
        yLimiter.calculate(speeds.vyMetersPerSecond),
        omegaLimiter.calculate(speeds.omegaRadiansPerSecond));
  }

  class PositiveSlewFilter {
    private double lastValue = 0;
    private double previousTime = 0;

    private double limiter;

    public PositiveSlewFilter(double limiter) {
      this.limiter = limiter;
    }

    public double calculate(double val) {

      if (previousTime == 0) {
        previousTime = Timer.getFPGATimestamp();
      }

      if (Math.abs(lastValue) > Math.abs(val)) {
        lastValue = val;
        previousTime = Timer.getFPGATimestamp();

        return val;
      }

      double currentTime = Timer.getFPGATimestamp();
      double elapsedTime = currentTime - previousTime;

      lastValue += MathUtil.clamp(val - lastValue, -limiter * elapsedTime, limiter * elapsedTime);

      return lastValue;
    }
  }
}
