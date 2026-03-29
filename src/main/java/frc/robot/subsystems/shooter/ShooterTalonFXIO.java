package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
// CHANGE PID VALUES !!!!
import frc.robot.Constants.ShooterConstants;

public class ShooterTalonFXIO implements ShooterIO {
  private final TalonFX mainMotor;
  private final TalonFX followerMotor;
  private final TalonFX followerMotor2;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final MotionMagicVelocityVoltage mVelocityRequest;

  private final StatusSignal<Temperature> temperatureCelsius;

  private double targetSpeed = 0;

  private boolean isBraked = true;

  // private PIDController PIDController = new PIDController(0.9, 0, 0.1);

  public ShooterTalonFXIO(int MainMotorCANID, int FollowerMotorCANID, int followerMotor2CANID) {
    mainMotor = new TalonFX(MainMotorCANID, Constants.MECHANISM_CANBUS);
    followerMotor = new TalonFX(FollowerMotorCANID, Constants.MECHANISM_CANBUS);
    followerMotor2 = new TalonFX(followerMotor2CANID, Constants.MECHANISM_CANBUS);

    // Basic Configuration
    TalonFXConfiguration config = Constants.ShooterConstants.CONFIG;
    Slot0Configs slot0Configs = Constants.ShooterConstants.SLOT0CONFIGS;

    mainMotor.getConfigurator().apply(config);
    mainMotor.getConfigurator().apply(slot0Configs);
    mainMotor.getConfigurator().apply(ShooterConstants.CLOSE_LOOP_RAMP_CONFIG);

    followerMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(slot0Configs);
    followerMotor.getConfigurator().apply(ShooterConstants.CLOSE_LOOP_RAMP_CONFIG);
    followerMotor.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    followerMotor2.getConfigurator().apply(config);
    followerMotor2.getConfigurator().apply(slot0Configs);
    followerMotor2.getConfigurator().apply(ShooterConstants.CLOSE_LOOP_RAMP_CONFIG);
    followerMotor2.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Initialize signals for AdvantageKit
    velocity = mainMotor.getVelocity();
    motorVoltage = mainMotor.getMotorVoltage();
    supplyCurrent = mainMotor.getSupplyCurrent();
    statorCurrent = mainMotor.getStatorCurrent();
    temperatureCelsius = mainMotor.getDeviceTemp();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.ShooterConstants.FREQUENCY_HZ,
        velocity,
        motorVoltage,
        supplyCurrent,
        statorCurrent,
        temperatureCelsius);

    mVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    isBraked = true;
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    BaseStatusSignal.refreshAll(velocity, motorVoltage, supplyCurrent, statorCurrent, temperatureCelsius);

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second
    inputs.voltage = motorVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.targetSpeed = targetSpeed;

    inputs.isBraked = isBraked;

    inputs.isBraked = isBraked;

    inputs.isOnTarget = isOnTargetSpeed();

    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setSpeed(double speed) {
    targetSpeed = speed;
    // motor.set(speed);
    mainMotor.setControl(mVelocityRequest.withVelocity(speed));
    // motor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setVoltage(double voltage) {
    // motor.setVoltage(voltage);
    mainMotor.setControl(new VoltageOut(voltage));
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

  @Override
  public void setBraked(boolean braked) {
    isBraked = braked;

    NeutralModeValue neutralModeValue;
    if (isBraked) {
      neutralModeValue = NeutralModeValue.Brake;
    } else {
      neutralModeValue = NeutralModeValue.Coast;
    }

    mainMotor.setNeutralMode(neutralModeValue);
  }
}
