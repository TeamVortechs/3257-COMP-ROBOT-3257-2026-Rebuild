package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
// CHANGE PID VALUES !!!!

public class IntakeTalonFXIO implements IntakeIO {

  private final TalonFX roller;
  private final TalonFX position;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerMotorVoltage;
  private final StatusSignal<Current> rollerSupplyCurrent;

  private final StatusSignal<AngularVelocity> positionVelocity;
  private final StatusSignal<Voltage> positionMotorVoltage;
  private final StatusSignal<Current> positionSupplyCurrent;

  private final MotionMagicVoltage mVoltageRequest;
  private double targetPosition = 0;

  public IntakeTalonFXIO(int canIdRoller, int canIdPosition) {
    roller = new TalonFX(canIdRoller);
    position = new TalonFX(canIdPosition);

    mVoltageRequest = new MotionMagicVoltage(0);

    // Basic Configuration
    TalonFXConfiguration config = Constants.IntakeConstants.CONFIG;
    Slot0Configs slot0Configs = Constants.IntakeConstants.SLOT0CONFIGS;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.MOTION_MAGIC_JERK;

    roller.getConfigurator().apply(config);
    position.getConfigurator().apply(config);
    roller.getConfigurator().apply(slot0Configs);
    position.getConfigurator().apply(slot0Configs);

    // Initialize signals for AdvantageKit
    rollerVelocity = roller.getVelocity();
    rollerMotorVoltage = roller.getMotorVoltage();
    rollerSupplyCurrent = roller.getSupplyCurrent();

    positionVelocity = position.getVelocity();
    positionMotorVoltage = position.getMotorVoltage();
    positionSupplyCurrent = position.getSupplyCurrent();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.FREQUENCY_HZ, rollerVelocity, rollerMotorVoltage, rollerSupplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.FREQUENCY_HZ, positionVelocity, positionMotorVoltage, positionSupplyCurrent);
  }

  // updates the given inputs with new values(advantage kit stuff)
  public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.rollerAmps = rollerSupplyCurrent.getValueAsDouble();
    inputsAutoLogged.rollerVolts = rollerMotorVoltage.getValueAsDouble();
    inputsAutoLogged.rollerSpeed = rollerVelocity.getValueAsDouble();

    inputsAutoLogged.positionAmps = positionSupplyCurrent.getValueAsDouble();
    inputsAutoLogged.positionVolts = positionMotorVoltage.getValueAsDouble();
    inputsAutoLogged.positionSpeed = positionVelocity.getValueAsDouble();

    inputsAutoLogged.position = position.getRotorPosition().getValueAsDouble();
    inputsAutoLogged.targetPosition = targetPosition;
  }

  // getters for motors

  // gets the height of the arm in meters
  public double getCurrent() {
    return rollerSupplyCurrent.getValueAsDouble();
  }

  public double getVoltage() {
    return rollerMotorVoltage.getValueAsDouble();
  }

  // setters for motors
  public void setRollerVoltage(double volt) {
    roller.setControl(new VoltageOut(volt));
  }

  public void setPositionVoltage(double volt) {
    position.setControl(new VoltageOut(volt));
  }

  // sets the position of the arm.
  public void setTargetPosition(double position1) { // IMPORTANT - POSITON1 NOT POSITION
    
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    position.setControl(mVoltageRequest.withPosition(position1));
    // System.out.println("Voltage being sent in PID Voltage");
  }

  public double getTargetPosition() {
    return targetPosition;
  }
  // misc methods

  public void resetEncoders() {
    position.setPosition(0);
  }

  public void setBraked(boolean braked) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    if (braked) {
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
    position.getConfigurator().apply(config);
  }

  // gets the highest possible height of the arm in radians
  public double getMaxPosition() {
    return Constants.IntakeConstants.MAX_POSITION;
  }

/**
 * @return gets the position of the arm in radians
 */
  public double getPosition() {
    return position.getRotorPosition().getValueAsDouble();
  }

  public boolean isMaxPosition() {
    return Math.abs(getPosition() - getMaxPosition())
        < Constants.IntakeConstants.POSITION_TOLERANCE;
  }
  
  public double getSpeed() {
    return roller.getVelocity().getValueAsDouble();
  }

  public boolean checkIfStalled() {
    return (roller.getMotorVoltage().getValueAsDouble() > Constants.IntakeConstants.ROLLER_STALLED_VOLTS);
  }
}
