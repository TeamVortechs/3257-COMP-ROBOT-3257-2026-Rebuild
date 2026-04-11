package frc.robot.subsystems.shooter;

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
import frc.robot.Constants;
// CHANGE PID VALUES !!!!
import frc.robot.Constants.ShooterConstants;

public class ShooterTalonFXIO implements ShooterIO {
  private final TalonFX mainMotor;
  private final TalonFX followerMotor;
  private final TalonFX followerMotor2;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> velocity;
  private final MotionMagicVelocityVoltage mVelocityRequest;

  private double targetSpeed = 0;

  private boolean isBraked = true;

  // private PIDController PIDController = new PIDController(0.9, 0, 0.1);

  public ShooterTalonFXIO(int MainMotorCANID, int FollowerMotorCANID, int followerMotor2CANID) {
    mainMotor = new TalonFX(MainMotorCANID);
    followerMotor = new TalonFX(FollowerMotorCANID);
    followerMotor2 = new TalonFX(followerMotor2CANID);

    // Basic Configuration
    TalonFXConfiguration config = Constants.ShooterConstants.CONFIG;
    Slot0Configs slot0Configs = Constants.ShooterConstants.SLOT0CONFIGS;

    mainMotor.getConfigurator().apply(config);
    mainMotor.getConfigurator().apply(slot0Configs);
    mainMotor.getConfigurator().apply(ShooterConstants.CLOSE_LOOP_RAMP_CONFIG);

    followerMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(slot0Configs);
    followerMotor.getConfigurator().apply(ShooterConstants.CLOSE_LOOP_RAMP_CONFIG);
    followerMotor.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    followerMotor2.getConfigurator().apply(config);
    followerMotor2.getConfigurator().apply(slot0Configs);
    followerMotor2.getConfigurator().apply(ShooterConstants.CLOSE_LOOP_RAMP_CONFIG);
    followerMotor2.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    // Initialize signals for AdvantageKit
    velocity = mainMotor.getVelocity();

    mVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    isBraked = true;
  }

  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    // Refresh signals from the hardware
    velocity.refresh();

    inputs.speed = velocity.getValueAsDouble(); // Returns Rotations per Second
    inputs.targetSpeed = targetSpeed;

    inputs.isBraked = isBraked;

    inputs.isBraked = isBraked;

    inputs.isOnTarget = isOnTargetSpeed();
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
