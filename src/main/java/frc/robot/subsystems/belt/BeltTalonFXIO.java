package frc.robot.subsystems.belt;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class BeltTalonFXIO implements BeltIO {
  private final TalonFX motor;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;

  private double targetSpeed = 0;

  private boolean isBraked = true;

  public BeltTalonFXIO(int canId) {
    motor = new TalonFX(canId);

    // Basic Configuration
    TalonFXConfiguration config = Constants.BeltConstants.CONFIG;

    motor.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    velocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    statorCurrent = motor.getStatorCurrent();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.FREQUENCY_HZ, velocity, motorVoltage, supplyCurrent);
  }

  @Override
  public void updateInputs(BeltIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    BaseStatusSignal.refreshAll(velocity, motorVoltage, supplyCurrent);

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second
    inputs.voltage = motorVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.targetOutput = targetSpeed;

    inputs.isBraked = isBraked;
  }

  @Override
  public void setPercentMotorOutput(double speed) {
    targetSpeed = speed;
    motor.set(speed);
    // motor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
    // motor.setControl(new VoltageOut(voltage));
  }

  @Override
  public double getSpeed() {
    return velocity.getValueAsDouble();
  }

  @Override
  public void setBraked(boolean braked) {

    isBraked = braked;
    
    NeutralModeValue neutralModeValue;

    if(isBraked) {
      neutralModeValue = NeutralModeValue.Brake;
    } else {
      neutralModeValue = NeutralModeValue.Coast;
    }

  
    motor.setNeutralMode(neutralModeValue);

  }
}
