package frc.robot.subsystems.canrange;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Canrange subsystem responsible for controlling the range finder. */
public class RangeFinder extends SubsystemBase {

  // advantage kit logging
  private RangeFinderIOInputsAutoLogged inputs = new RangeFinderIOInputsAutoLogged();

  // useful for a flexible hardware interface and for advantage kit logging
  private final RangeFinderIO moduleIO;

  /**
   * Constructor for the Canrange subsystem.
   *
   * @param moduleIO Hardware interface for Canrange.
   */
  public RangeFinder(RangeFinderIO moduleIO) {
    this.moduleIO = moduleIO;
  }

  @Override
  public void periodic() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Canrange", inputs);

    // check to see if the module is stalling; if so, then stop the motors and cancel the next
    // movement
  }

  // gets the distance of the can Range
  public double getCanDistance() {
    return moduleIO.getDistance();
  }

  // sets canrange distance for simulation
  public void setCanRangeDistanceSimulation(double distance) {
    moduleIO.setCanrangeDistance(distance);
  }

  // commands
  // simple command that requires this subsystem
  public Command requireSubsystemCommand() {
    return new InstantCommand(() -> {}, this);
  }

  // sets the canrange distance for simulation
  public Command setCanrangeDistanceCommand(double dist) {
    return new InstantCommand(() -> setCanRangeDistanceSimulation(dist));
  }
}
