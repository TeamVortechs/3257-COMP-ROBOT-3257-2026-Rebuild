package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PowerModuleIORev implements PowerModuleIO {
  public final PowerDistribution PDH;
  private boolean pdhEnabled;

  public PowerModuleIORev() {
    PDH = new PowerDistribution(1, ModuleType.kRev);
    pdhEnabled = true;
  }

  @Override
  public void updateInputs(PowerModuleIOInputsAutoLogged inputs) {
    inputs.voltage = PDH.getVoltage();
    inputs.temp = PDH.getTemperature();
    inputs.current = PDH.getTotalCurrent();
    inputs.power = PDH.getTotalPower();
    inputs.energy = PDH.getTotalEnergy();

    inputs.PDHEnabled = pdhEnabled;
  }

  public void setPDH(boolean enabled) {
    PDH.setSwitchableChannel(enabled);
    pdhEnabled = enabled;
  }
}
