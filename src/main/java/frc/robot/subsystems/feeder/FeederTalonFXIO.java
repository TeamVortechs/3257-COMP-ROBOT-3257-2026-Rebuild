package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;

public class FeederTalonFXIO implements FeederIO {
  private final TalonFX motor;

  // private final VelocityVoltage mVelocityRequest;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperatureCelsius;

  private double targetSpeed = 0;

  private boolean isBraked = true;

  public FeederTalonFXIO(int canId) {
    motor = new TalonFX(canId);
    // mVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Basic Configuration
    TalonFXConfiguration config = Constants.FeederConstants.CONFIG;

    Slot0Configs slot0Configs = Constants.FeederConstants.SLOT0CONFIGS;

    motor.getConfigurator().apply(slot0Configs);

    motor.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    velocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    statorCurrent = motor.getStatorCurrent();
    temperatureCelsius = motor.getDeviceTemp();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        FeederConstants.SUBSYSTEM_LOGGING_FREQUENCY_HERTZ,
        velocity,
        motorVoltage,
        supplyCurrent,
        statorCurrent,
        temperatureCelsius);

    isBraked = true;
  }

  @Override
  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    BaseStatusSignal.refreshAll(velocity, motorVoltage, supplyCurrent);

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second
    inputs.voltage = motorVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.targetSpeed = targetSpeed;

    inputs.isOnTargetSpeed = isOnTargetSpeed();

    inputs.isBraked = isBraked;

    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setPercentMotorOutput(double speed) {
    targetSpeed = speed;

    // motor.set(speed);
    motor.set(speed);
    // motor.setControl(new DutyCycleOut(speed));
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void setVoltage(double voltage) {
    // motor.setVoltage(voltage);
    motor.setControl(new VoltageOut(voltage));
    // motor.setControl(new VoltageOut(voltage));
  }

  @Override
  public double getSpeed() {
    return velocity.getValueAsDouble();
  }

  @Override
  public boolean isOnTargetSpeed() {
    return Math.abs(getSpeed() - targetSpeed) < Constants.FeederConstants.POSITION_TOLERANCE;
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
