package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class FeederTalonFXIO implements FeederIO {
  private final TalonFX motor;

  // private final VelocityVoltage mVelocityRequest;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> velocity;

  private boolean isBraked = true;

  public FeederTalonFXIO(int canId) {
    motor = new TalonFX(canId, Constants.MECHANISM_CANBUS);
    // mVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Basic Configuration
    TalonFXConfiguration config = Constants.FeederConstants.CONFIG;

    Slot0Configs slot0Configs = Constants.FeederConstants.SLOT0CONFIGS;

    motor.getConfigurator().apply(slot0Configs);

    motor.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    velocity = motor.getVelocity();

    isBraked = true;
  }

  @Override
  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    velocity.refresh();

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second

    inputs.isBraked = isBraked;
  }

  @Override
  public void setPercentMotorOutput(double speed) {

    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(new VoltageOut(voltage));
  }

  @Override
  public double getSpeed() {
    return velocity.getValueAsDouble();
  }

  @Override
  public void setBraked(boolean braked) {
    isBraked = braked;

    NeutralModeValue neutralModeValue;

    if (braked) {
      neutralModeValue = NeutralModeValue.Brake;
    } else {
      neutralModeValue = NeutralModeValue.Coast;
    }

    motor.setNeutralMode(neutralModeValue);
  }
}
