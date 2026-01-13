package frc.robot.subsystems.canrange;

public class RangeFinderSimulationIO implements RangeFinderIO {

  private double distance;

  public void updateInputs(RangeFinderIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.canRangeDistance = distance;
  }

  public void setCanrangeDistance(double dist) {
    this.distance = dist;
  }

  // gets the distance of the can rnage
  public double getDistance() {
    return distance;
  }
}
