package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  @AutoLogOutput private boolean isLocked = true;
  @AutoLogOutput private double targetPosition = 0.0;

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  /** set servo in degrees */
  public void setServo(double position) {
    climbIO.setServo(position);
  }

  /**
   * Manual control for both motors. Useful if the robot is tilting and you need to adjust one side.
   */
  public void setManualSpeeds(double left, double right) {
    if (isLocked) {
      climbIO.stop();
      return;
    }
    climbIO.setSpeeds(left, right);
  }

  public void stop() {
    climbIO.stop();
  }
}
