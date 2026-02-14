package frc.robot.subsystems.LED_strip;

import com.ctre.phoenix6.controls.SolidColor;

public interface LEDStripIO {

  public default void setColor(SolidColor color) {}
  ;
}
