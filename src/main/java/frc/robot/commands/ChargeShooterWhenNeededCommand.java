package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;

/*
Names
brief description
 */
public class ChargeShooterWhenNeededCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private BooleanSupplier isInChargeZone;

  private Shooter shooter;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChargeShooterWhenNeededCommand(Shooter shooter, BooleanSupplier isInChargeZone) {
    addRequirements(shooter);

    this.shooter = shooter;
    this.isInChargeZone = isInChargeZone;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // simple zone check to see if it should charge yet

    // if it's within zone it should be charged reasonably close because we're most likely gonna
    // shoot soon
    if (isInChargeZone.getAsBoolean()) {
      shooter.setAutomatic(ShooterConstants.PERCENTAGE_OF_DISTANCE_WHEN_CHARGING);
    } else {
      // if it's not in zone it shoudl be pretty small to conserve poewr
      shooter.setManualSpeed(ShooterConstants.DEFAULT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
