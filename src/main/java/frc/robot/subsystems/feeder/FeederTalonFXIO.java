package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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

  private final MotionMagicVelocityVoltage mVelocityRequest;

  private double targetSpeed = 0;

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

    mVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
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

  @Override
  public void setSpeed(double speed) {
    motor.setControl(mVelocityRequest.withVelocity(speed));
    targetSpeed = speed;
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
  public double getTargetSpeed() {
    return targetSpeed;
  }

  public boolean isOnTargetSpeed() {
    return Math.abs(this.getSpeed() - targetSpeed) < Constants.ShooterConstants.TOLERANCE;
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
