package frc.robot.subsystems.LED_strip;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

public class LEDStripTalonFXIO implements LEDStripIO {
  private CANdle candle;

  // for animation
  private final CANBus kCANBus;
  private final CANdle m_candle;
  private final ColorFlowAnimation m_slot0Animation;

  public LEDStripTalonFXIO(int id) {
    // for blinking
    candle = new CANdle(id);

    // for animation
    kCANBus = new CANBus("rio");
    m_candle = new CANdle(10, kCANBus);
    m_slot0Animation =
        new ColorFlowAnimation(0, 7)
            .withSlot(0)
            .withColor(new RGBWColor(255, 179, 26, 0))
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(Hertz.of(25));
  }

  @Override
  public void setColor(SolidColor color) {
    candle.setControl(color);
  }
}
