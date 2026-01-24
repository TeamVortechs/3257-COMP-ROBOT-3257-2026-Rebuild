package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  @AutoLogOutput private boolean isLocked = true;

  @AutoLogOutput private boolean isManual = false;

  @AutoLogOutput private double manualLeftSpeed = 0;
  @AutoLogOutput private double manualRightSpeed = 0;

  @AutoLogOutput private double automaticLeftSetpoint = 0;
  @AutoLogOutput private double automaticRightSetpoint = 0;

  public Climb(ClimbIO climbIO) {
    this.climbIO = climbIO;
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    if (isLocked) {
      climbIO.stop();

      setManualSpeeds(0, 0);
      return;
    }

    if (isManual) {
      climbIO.setSpeeds(0, 0);
    } else {
      climbIO.setPositions(automaticLeftSetpoint, automaticRightSetpoint);
    }
  }

  /** set servo in degrees */
  public void setServo(double position) {
    climbIO.setServo(position);
  }

  /**
   * Manual control for both motors. Useful if the robot is tilting and you need to adjust one side.
   * sets it to manual
   */
  public void setManualSpeeds(double left, double right) {
    // if (isLocked) {
    //   climbIO.stop();
    //   return;
    // }

    isManual = true;

    climbIO.setSpeeds(left, right);
  }

  /**
   * sets the climber to automatic and sets the positions
   * @param leftPosition
   * @param rightPosition
   */
  public void setPositions(double leftPosition, double rightPosition) {
    isManual = false;
    this.automaticLeftSetpoint = leftPosition;
    this.automaticRightSetpoint = rightPosition;
  } 

  /**
   * makes the climber manual and stops the setpoint
   */
  public void stop() {
    climbIO.stop();
  }
}
