package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
// CHANGE PID VALUES !!!!

public class ShooterTalonFXIO implements ShooterIO {
  private final TalonFX motor;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final VelocityVoltage mVelocityRequest;
  private double targetSpeed = 0;

  // private PIDController PIDController = new PIDController(0.9, 0, 0.1);

  public ShooterTalonFXIO(int canId) {
    motor = new TalonFX(canId);

    // Basic Configuration
    TalonFXConfiguration config = Constants.ShooterConstants.CONFIG;
    Slot0Configs slot0Configs = Constants.ShooterConstants.SLOT0CONFIGS;

    motor.getConfigurator().apply(config);

    motor.getConfigurator().apply(slot0Configs);

    // Initialize signals for AdvantageKit
    velocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.FREQUENCY_HZ, velocity, motorVoltage, supplyCurrent);

    mVelocityRequest = new VelocityVoltage(0).withSlot(0);
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    BaseStatusSignal.refreshAll(velocity, motorVoltage, supplyCurrent);

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second
    inputs.voltage = motorVoltage.getValueAsDouble();
    inputs.amps = supplyCurrent.getValueAsDouble();
    inputs.targetSpeed = targetSpeed;

    inputs.isOnTarget = isOnTargetSpeed();
  }

  @Override
  public void setSpeed(double speed) {
    targetSpeed = speed;
    // motor.set(speed);
    motor.setControl(mVelocityRequest.withVelocity(speed));
    // motor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setVoltage(double voltage) {
    // motor.setVoltage(voltage);
    motor.setControl(new VoltageOut(voltage));
    // motor.setControl(new DutyCycleOut(voltage));
  }

  @Override
  public double getSpeed() {
    return velocity.getValueAsDouble();
  }

  @Override
  public boolean isOnTargetSpeed() {
    return Math.abs(this.getSpeed() - targetSpeed) < Constants.ShooterConstants.TOLERANCE;
  }
}
