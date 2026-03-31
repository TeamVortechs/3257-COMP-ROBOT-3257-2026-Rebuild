package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface PowerModuleIO {
  // private final PDH;

  @AutoLog
  public static class PowerModuleIOInputs {
    public double voltage = 0;
    public double temp = 0;
    public double current = 0;
    public double power = 0;
    public double energy = 0;
  }

  public default void setPDH(boolean enabled){}

  public default void updateInputs(PowerModuleIOInputsAutoLogged inputs) {}
}
