package frc.robot.subsystems.LED_strip;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

public class LEDStripTalonFXIO implements LEDStripIO {
  private CANdle candle;

  public LEDStripTalonFXIO(int id) {
    candle = new CANdle(id);
  }

  @Override
  public void setColor(SolidColor color) {
    candle.setControl(color);
  }
}
