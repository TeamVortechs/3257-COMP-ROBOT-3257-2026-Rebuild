package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PowerModuleIORev implements PowerModuleIO {
  public final PowerDistribution PDH;

  public PowerModuleIORev() {
    PDH = new PowerDistribution(1, ModuleType.kRev);
  }

  @Override
  public void updateInputs(PowerModuleIOInputsAutoLogged inputs) {
    inputs.voltage = PDH.getVoltage();
    inputs.temp = PDH.getTemperature();
    inputs.current = PDH.getTotalCurrent();
    inputs.power = PDH.getTotalPower();
    inputs.energy = PDH.getTotalEnergy();
  }

  public void setPDH(boolean enabled){
    PDH.setSwitchableChannel(enabled);
  }
}
