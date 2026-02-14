package frc.robot.subsystems.LED_strip;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {

  private LEDStripIO ledStripIO;

  private Timer timer;

  private boolean orange = false;

  private SolidColor orangeColor = new SolidColor(0, 7);
  private SolidColor whiteColor = new SolidColor(0, 7);

  public LEDStrip(LEDStripIO ledStripIO) {
    this.ledStripIO = ledStripIO;
    timer = new Timer();
    timer.reset();

    orangeColor.Color = new RGBWColor(Color.kOrange);

    whiteColor.Color = new RGBWColor(Color.kWhite);
  }

  @Override
  public void periodic() {
    if (timer.advanceIfElapsed(1)) {
      if (orange) {
        ledStripIO.setColor(orangeColor);
      } else {
        ledStripIO.setColor(whiteColor);
      }

      orange = !orange;
      System.out.println("running switch");
    }
  }
}
