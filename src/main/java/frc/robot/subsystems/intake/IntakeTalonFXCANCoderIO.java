package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeTalonFXCANCoderIO implements IntakeIO {
  private final TalonFX roller;
  private final TalonFX rollerFollower;

  private final TalonFX position;
  private final CANcoder caNcoder;

  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Angle> intakePosition;

  private final MotionMagicVoltage mVoltageRequest;

  private boolean isBrakedRoller = false;
  private boolean isBrakedPosition = true;
  private double targetPosition = 0;

  public IntakeTalonFXCANCoderIO(
      int canIdRoller, int canIDRoller2, int canIdPosition, int canIdCANCoder) {
    roller = new TalonFX(canIdRoller, Constants.MECHANISM_CANBUS);
    rollerFollower = new TalonFX(canIDRoller2, Constants.MECHANISM_CANBUS);
    position = new TalonFX(canIdPosition, Constants.MECHANISM_CANBUS);
    caNcoder = new CANcoder(canIdCANCoder, Constants.MECHANISM_CANBUS);

    mVoltageRequest = new MotionMagicVoltage(0);

    TalonFXConfiguration rollerConfig = Constants.IntakeConstants.ROLLER_CONFIG;
    TalonFXConfiguration positionConfig = Constants.IntakeConstants.POSITION_CONFIG;
    Slot0Configs slot0Configs = Constants.IntakeConstants.SLOT0CONFIGS;

    var motionMagicConfigs = rollerConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration =
        Constants.IntakeConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.MOTION_MAGIC_JERK;

    positionConfig.Feedback =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(canIdCANCoder)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

    positionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    positionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        IntakeConstants.MAX_POSITION - IntakeConstants.POSITION_THRESHOLD_STOP;

    positionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    positionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        IntakeConstants.MIN_POSITION + IntakeConstants.POSITION_THRESHOLD_STOP;

    // Replace VortechsUtil.clamp with Hardware Voltage Limits
    positionConfig.Voltage.PeakForwardVoltage = IntakeConstants.CLAMP_MAX_VOLTS;
    positionConfig.Voltage.PeakReverseVoltage = -IntakeConstants.CLAMP_MAX_VOLTS;

    caNcoder.getConfigurator().apply(IntakeConstants.CANCODER_CONFIG);

    roller.getConfigurator().apply(rollerConfig);
    rollerFollower.getConfigurator().apply(rollerConfig);
    rollerFollower.setControl(new Follower(roller.getDeviceID(), MotorAlignmentValue.Opposed));

    position.getConfigurator().apply(positionConfig);
    position.getConfigurator().apply(motionMagicConfigs);
    position.getConfigurator().apply(slot0Configs);

    // Initialize signals for AdvantageKit
    rollerVelocity = roller.getVelocity();
    intakePosition = position.getPosition();
  }

  public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {

    BaseStatusSignal.refreshAll(rollerVelocity, intakePosition);

    inputsAutoLogged.rollerSpeed = rollerVelocity.getValueAsDouble();

    inputsAutoLogged.position = intakePosition.getValueAsDouble();
    inputsAutoLogged.targetPosition = targetPosition;

    inputsAutoLogged.isOnTarget =
        Math.abs(targetPosition - inputsAutoLogged.position) < IntakeConstants.POSITION_TOLERANCE;
  }

  // setters for motors
  public void setRollerVoltage(double volt) {
    roller.setControl(new VoltageOut(volt));
  }

  public void setPositionVoltage(double volts) {

    position.setControl(new VoltageOut(volts));
  }

  // sets the position of the arm.
  public void setPositionControl(double position1) { // IMPORTANT - POSITON1 NOT POSITION
    targetPosition = position1;

    // dumb attempt at workaround due to no CANivore
    var motionMagicConfigs = IntakeConstants.POSITION_CONFIG.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    position.getConfigurator().apply(motionMagicConfigs);

    position.setControl(mVoltageRequest.withPosition(position1));
  }

  public void setPositionControlWithVelocity(
      double position1, double velocity) { // IMPORTANT - POSITON1 NOT POSITION
    targetPosition = position1;

    // dumb attempt at workaround due to no CANivore
    var motionMagicConfigs = IntakeConstants.POSITION_CONFIG.MotionMagic.clone();
    motionMagicConfigs.MotionMagicCruiseVelocity = velocity;
    position.getConfigurator().apply(motionMagicConfigs);

    position.setControl(mVoltageRequest.withPosition(position1));
  }

  public void resetEncoder(double positionVal) {
    caNcoder.setPosition(positionVal);
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
    return intakePosition.getValueAsDouble();
  }

  public double getRollerSpeed() {
    return rollerVelocity.getAppliedUpdateFrequency();
  }
}
