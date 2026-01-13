package frc.robot.subsystems.canrange;

import com.ctre.phoenix6.hardware.CANrange;

public class RangeFinderIOCanrange implements RangeFinderIO {

  private CANrange canrange;

  public RangeFinderIOCanrange(int canrangeID, String canbusName) {
    this.canrange = new CANrange(canrangeID, canbusName);
  }

  public void updateInputs(RangeFinderIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.canRangeDistance = getDistance();
  }

  // gets the distance of the can rnage
  public double getDistance() {
    return canrange.getDistance().getValueAsDouble();
  }
}
