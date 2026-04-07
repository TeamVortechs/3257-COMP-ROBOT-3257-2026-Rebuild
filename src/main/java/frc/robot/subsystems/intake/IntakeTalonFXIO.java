package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeTalonFXIO implements IntakeIO {
  private final TalonFX roller;
  private final TalonFX position;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerMotorVoltage;
  private final StatusSignal<Current> rollerStatorCurrent;
  private final StatusSignal<Current> rollerSupplyCurrent;

  private final StatusSignal<AngularVelocity> positionVelocity;
  private final StatusSignal<Voltage> positionMotorVoltage;
  private final StatusSignal<Current> positionStatorCurrent;
  private final StatusSignal<Current> positionSupplyCurrent;

  private final StatusSignal<Temperature> positionTemperatureCelsius;
  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private final DynamicMotionMagicVoltage mVoltageRequest;

  // private final PositionVoltage mPositionVoltage;

  private boolean isBrakedRoller = false;
  private boolean isBrakedPosition = true;
  private double targetPosition = 0;

  public IntakeTalonFXIO(int canIdRoller, int canIdPosition) {
    roller = new TalonFX(canIdRoller);
    position = new TalonFX(canIdPosition);

    mVoltageRequest =
        new DynamicMotionMagicVoltage(
                0,
                IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY,
                IntakeConstants.MOTION_MAGIC_ACCELERATION)
            .withJerk(IntakeConstants.MOTION_MAGIC_JERK);
    // mPositionVoltage = new PositionVoltage(0);

    // 1.29
    // 2
    // Basic Configuration
    TalonFXConfiguration rollerConfig = Constants.IntakeConstants.ROLLER_CONFIG;
    TalonFXConfiguration positionConfig = Constants.IntakeConstants.POSITION_CONFIG;
    Slot0Configs slot0Configs = Constants.IntakeConstants.SLOT0CONFIGS;

    var motionMagicConfigs = rollerConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration =
        Constants.IntakeConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.MOTION_MAGIC_JERK;

    positionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    positionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.MAX_POSITION - IntakeConstants.POSITION_THRESHOLD_STOP;

    positionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    positionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.MIN_POSITION + IntakeConstants.POSITION_THRESHOLD_STOP;

    // Replace VortechsUtil.clamp with Hardware Voltage Limits
    positionConfig.Voltage.PeakForwardVoltage = IntakeConstants.CLAMP_MAX_VOLTS;
    positionConfig.Voltage.PeakReverseVoltage = -IntakeConstants.CLAMP_MAX_VOLTS;

    roller.getConfigurator().apply(rollerConfig);
    position.getConfigurator().apply(positionConfig);
    position.getConfigurator().apply(motionMagicConfigs);

    position.getConfigurator().apply(slot0Configs);

    // Initialize signals for AdvantageKit
    rollerVelocity = roller.getVelocity();
    rollerMotorVoltage = roller.getMotorVoltage();
    rollerStatorCurrent = roller.getStatorCurrent();
    rollerSupplyCurrent = roller.getSupplyCurrent();
    rollerTemperatureCelsius = roller.getDeviceTemp();

    positionVelocity = position.getVelocity();
    positionMotorVoltage = position.getMotorVoltage();
    positionStatorCurrent = position.getStatorCurrent();
    positionSupplyCurrent = position.getSupplyCurrent();
    positionTemperatureCelsius = position.getDeviceTemp();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.IntakeConstants.FREQUENCY_HZ,
        rollerVelocity,
        rollerMotorVoltage,
        rollerStatorCurrent,
        rollerSupplyCurrent,
        rollerTemperatureCelsius);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.IntakeConstants.FREQUENCY_HZ,
        positionVelocity,
        positionMotorVoltage,
        positionStatorCurrent,
        positionSupplyCurrent,
        positionTemperatureCelsius);
  }

  public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.rollerAmpsStator = rollerStatorCurrent.getValueAsDouble();
    inputsAutoLogged.rollerAmpsSupply = rollerSupplyCurrent.getValueAsDouble();
    inputsAutoLogged.rollerVolts = rollerMotorVoltage.getValueAsDouble();
    inputsAutoLogged.rollerSpeed = rollerVelocity.getValueAsDouble();
    inputsAutoLogged.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputsAutoLogged.positionAmpsStator = positionStatorCurrent.getValueAsDouble();
    inputsAutoLogged.positionAmpsSupply = positionSupplyCurrent.getValueAsDouble();
    inputsAutoLogged.positionVolts = positionMotorVoltage.getValueAsDouble();
    inputsAutoLogged.positionSpeed = positionVelocity.getValueAsDouble();
    inputsAutoLogged.positionTemperatureCelsius = positionTemperatureCelsius.getValueAsDouble();

    inputsAutoLogged.position = position.getPosition().getValueAsDouble();
    inputsAutoLogged.targetPosition = targetPosition;

    inputsAutoLogged.isBrakedRoller = isBrakedRoller;
    inputsAutoLogged.isBrakedPosition = isBrakedPosition;
  }

  // getters for motors

  public double getRollerCurrent() {
    return rollerStatorCurrent.getValueAsDouble();
  }

  public double getVoltage() {
    return rollerMotorVoltage.getValueAsDouble();
  }

  // setters for motors
  public void setRollerVoltage(double volt) {
    roller.setControl(new VoltageOut(volt));
  }

  public void setPositionVoltage(double volts) {

    // replaced all this with limit logic at initialization

    // VortechsUtil.clamp(volts, IntakeConstants.CLAMP_MAX_VOLTS);

    // if (volts > 0
    //     && getPosition() > IntakeConstants.MAX_POSITION -
    // IntakeConstants.POSITION_THRESHOLD_STOP) {
    //   volts = 0;
    // }

    // if (volts < 0
    //     && getPosition() < IntakeConstants.MIN_POSITION +
    // IntakeConstants.POSITION_THRESHOLD_STOP) {
    //   volts = 0;
    // }

    position.setControl(new VoltageOut(volts));
  }

  // sets the position of the arm.
  public void setPositionControl(double position1) { // IMPORTANT - POSITON1 NOT POSITION
    targetPosition = position1;

    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    position.setControl(mVoltageRequest.withPosition(position1));
    // System.out.println("Voltage being sent in PID Voltage");
  }

  public void setPositionControlWithVelocity(
      double position1, double velocity) { // IMPORTANT - POSITON1 NOT POSITION
    targetPosition = position1;

    // System.out.println("VERY SLOWLY setting position in FXIO to " + position1);
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    position.setControl(mVoltageRequest.withPosition(position1).withVelocity(velocity));
    // System.out.println("Voltage being sent in PID Voltage");
  }

  public void resetEncoder(double positionVal) {
    position.setPosition(positionVal);
  }

  public void resetEncoders() {
    resetEncoder(IntakeConstants.MAX_POSITION);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void stop() {
    roller.stopMotor();
    position.stopMotor();
  }

  // misc methods

  public void setBrakedRoller(boolean braked) {
    isBrakedRoller = braked;

    NeutralModeValue neutralModeValue;

    if (braked) {
      neutralModeValue = NeutralModeValue.Brake;
    } else {
      neutralModeValue = NeutralModeValue.Coast;
    }

    roller.setNeutralMode(neutralModeValue);
  }

  public void setBrakedPosition(boolean braked) {
    isBrakedPosition = braked;

    NeutralModeValue neutralModeValue;

    if (braked) {
      neutralModeValue = NeutralModeValue.Brake;
    } else {
      neutralModeValue = NeutralModeValue.Coast;
    }

    roller.setNeutralMode(neutralModeValue);
  }

  /**
   * @return gets the position of the arm in radians
   */
  public double getPosition() {
    return position.getPosition().getValueAsDouble();
  }

  /**
   * @return Gets the speed of the roller in rotations/sec
   */
  public double getRollerSpeed() {
    return roller.getVelocity().getValueAsDouble();
  }

  // public boolean checkIfStalled() {
  //   return (roller.getMotorVoltage().getValueAsDouble()
  //       > Constants.IntakeConstants.ROLLER_STALLED_VOLTS);
  // }

  public double getRollerMotorVoltage() {
    return roller.getMotorVoltage().getValueAsDouble();
  }

  public boolean isRollerJammed() {
    // according to google, having a high current + low speed + voltage applied = JAM
    boolean rollerJammed =
        Math.abs(getRollerCurrent()) > IntakeConstants.ROLLER_JAM_CURRENT_AMPS
            && Math.abs(getRollerSpeed()) < IntakeConstants.ROLLER_JAM_VELOCITY
            && Math.abs(getRollerMotorVoltage()) > IntakeConstants.MIN_VOLTAGE_APPLIED;

    return rollerJammed;
  }

  public boolean isPositionJammed() {
    // high current + far from target + voltage applied = JAM
    boolean positionJammed =
        Math.abs(positionStatorCurrent.getValueAsDouble())
                > IntakeConstants.POSITION_JAM_CURRENT_AMPS
            && Math.abs(getTargetPosition() - getPosition()) > IntakeConstants.POSITION_TOLERANCE
            && Math.abs(positionMotorVoltage.getValueAsDouble()) > 2.0;

    return positionJammed;
  }
}
