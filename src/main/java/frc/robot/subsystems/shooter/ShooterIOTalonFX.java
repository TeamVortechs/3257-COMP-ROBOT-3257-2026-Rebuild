package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {
    TalonFX talonFX;

    public ShooterIOTalonFX(int motorID, String canbusName) {
        talonFX = new TalonFX(motorID, canbusName);
    }

    public void updateInputs(ShooterIOInputsAutoLogged inputs) {

    }

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
