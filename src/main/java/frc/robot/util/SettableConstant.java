package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// constant that uses smartdashboard so u can set it
public class SettableConstant {

  private double defaultValue;
  private String key;

  public SettableConstant(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;

    SmartDashboard.putNumber(key, defaultValue);
  }

  public double get() {

    return SmartDashboard.getNumber(key, defaultValue);
  }
}
