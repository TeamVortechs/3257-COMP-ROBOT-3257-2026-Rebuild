package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;

public class VortechsUtil {
  public static double clamp(double num, double clampVal) {
    return clamp(num, -clampVal, clampVal);
  }

  public static double clamp(double num, double lowerClamp, double higherClamp) {
    return Math.min(higherClamp, Math.max(num, lowerClamp));
  }

  public static boolean isInTolerance(double num1, double tolerance) {
    return Math.abs(num1) < tolerance;
  }

  public static boolean isWithinXZone(double x, boolean wantsCenter, Pose2d pose) {
    double xPose = pose.getX();

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

  public static boolean isWithinYZone(double y, boolean wantsLeft, Pose2d pose) {
    double yPose = pose.getY();

    if (DriverStation.getAlliance().isEmpty()) {
      return false;
    }

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      if (!wantsLeft) {
        return yPose < y;
      } else {
        return yPose > y;
      }
    } else {
      if (!wantsLeft) {
        return yPose > (2 * DriveConstants.CENTER_POINT.getY() - y);
      } else {
        return yPose < (2 * DriveConstants.CENTER_POINT.getY() - y);
      }
    }
  }

  public static Supplier<Pose2d> AllianceBasedPose(Pose2d bluePose, Pose2d redPose) {
    return () -> {
      if (DriverStation.getAlliance() == null || DriverStation.getAlliance().isEmpty()) {
        return bluePose;
      }

      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return bluePose;
      } else {
        return redPose;
      }
    };
  }
}
