package frc.robot.subsystems.belt;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class BeltTalonFXIO implements BeltIO {
  private final TalonFX motor;

  private boolean isBraked = true;

  public BeltTalonFXIO(int canId) {
    motor = new TalonFX(canId);

    // Basic Configuration
    TalonFXConfiguration config = Constants.BeltConstants.CONFIG;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(BeltIOInputsAutoLogged inputs) {}

  @Override
  public void setPercentMotorOutput(double speed) {
    motor.set(speed);
    // motor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setBraked(boolean braked) {

    isBraked = braked;

    NeutralModeValue neutralModeValue;

    if (isBraked) {
      neutralModeValue = NeutralModeValue.Brake;
    } else {
      neutralModeValue = NeutralModeValue.Coast;
    }

    motor.setNeutralMode(neutralModeValue);
  }
}
