package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class VortechsController extends CommandXboxController {

  private double rumbleValue;

  public VortechsController(int port) {
    super(port);
    rumbleValue = 0;
  }

  @Override
  public void setRumble(GenericHID.RumbleType type, double value) {
    super.setRumble(type, value);
    rumbleValue = value;
  }

  public boolean isRumbling() {
    return rumbleValue > 0;
  }

  public double getRumbleValue() {
    return rumbleValue;
  }

  public Command setRumbleCommand(double amplitude) {
    return new InstantCommand(() -> this.setRumble(GenericHID.RumbleType.kBothRumble, amplitude));
  }

  public Command setRumbleRunCommand(double amplitude) {
    return new RunCommand(() -> this.setRumble(GenericHID.RumbleType.kBothRumble, amplitude))
        .finallyDo(() -> this.setRumble(GenericHID.RumbleType.kBothRumble, 0));
  }
}
