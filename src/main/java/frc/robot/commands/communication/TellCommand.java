package frc.robot.commands.communication;

import edu.wpi.first.wpilibj2.command.Command;

/*
Names
brief description
 */
public class TellCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private String message;

  private boolean messaged = false;

  public TellCommand(String message) {
    // addRequirements(null);
    this.message = message;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    messaged = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(message);
    messaged = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return messaged;
  }
}
