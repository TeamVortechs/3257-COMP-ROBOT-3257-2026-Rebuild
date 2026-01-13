package frc.robot.subsystems.canrange;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for arm Module IO implementations. This abstracts all hardware interactions for the
 * arm.
 */
public interface RangeFinderIO {
  @AutoLog
  public static class RangeFinderIOInputs {
    double canRangeDistance = 0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(RangeFinderIOInputsAutoLogged inputsAutoLogged) {}

  // canrange functions
  // sets the distance from the can range. This is only used in simulation for system testing and
  // doesn't work on real ios
  public default void setCanrangeDistance(double dist) {}

  // gets the distance of the can rnage
  public default double getDistance() {
    return 0;
  }
}
