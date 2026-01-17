package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {
  TalonFX talonFX;

  public ShooterIOTalonFX(int motorID, CANBus canBus) {
    talonFX = new TalonFX(motorID, canBus);
  }

  public void updateInputs(ShooterIOInputsAutoLogged inputs) {}

  public void setSpeed(double speed) {
    talonFX.set(speed);
  }

  public double getSpeed() {
    return talonFX.get();
  }

  public void setVoltage(double voltage) {
    talonFX.setVoltage(voltage);
  }
}
