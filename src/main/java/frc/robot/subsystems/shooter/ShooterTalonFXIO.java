package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private double targetSpeed = 0;

  // private PIDController PIDController = new PIDController(0.9, 0, 0.1);

  public ShooterTalonFXIO(int canId) {
    motor = new TalonFX(canId);

    // Basic Configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 40.0; // Prevent breaker trips
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = Constants.ShooterConstants.kS; // Add 0.1 V output to overcome static friction
    slot0Configs.kV =
        Constants.ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = Constants.ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = Constants.ShooterConstants.kI; // no output for integrated error
    slot0Configs.kD = Constants.ShooterConstants.kD; // no output for error derivative

    motor.getConfigurator().apply(config);

    motor.getConfigurator().apply(slot0Configs);

    // Initialize signals for AdvantageKit
    velocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, motorVoltage, supplyCurrent);
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    BaseStatusSignal.refreshAll(velocity, motorVoltage, supplyCurrent);

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second
    inputs.voltage = motorVoltage.getValueAsDouble();
    inputs.amps = supplyCurrent.getValueAsDouble();
    inputs.targetSpeed = targetSpeed;

    inputs.isOnTarget = isOnTarget();
  }

  @Override
  public void setSpeed(double speed) {
    targetSpeed = speed;
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    // motor.set(speed);
    motor.setControl(m_request.withVelocity(speed));
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
  public boolean isOnTarget() {
    return Math.abs(this.getSpeed() - (targetSpeed * 100)) < 5;
  }
}
