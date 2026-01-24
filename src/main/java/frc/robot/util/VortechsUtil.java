package frc.robot.util;

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
}
