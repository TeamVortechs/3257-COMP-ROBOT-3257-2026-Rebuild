package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

  private double targetPosition = 0;

  public IntakeTalonFXIO(int canIdRoller, int canIdPosition) {
    roller = new TalonFX(canIdRoller);
    position = new TalonFX(canIdPosition);

    // Basic Configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    var slot0Configs = config.Slot0;
    slot0Configs.kS = Constants.IntakeConstants.kS;
    slot0Configs.kV = Constants.IntakeConstants.kV;
    slot0Configs.kA = Constants.IntakeConstants.kA;
    slot0Configs.kP = Constants.IntakeConstants.kP;
    slot0Configs.kI = Constants.IntakeConstants.kI;
    slot0Configs.kD = Constants.IntakeConstants.kD;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.IntakeConstants.MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.MotionMagicJerk;

    roller.getConfigurator().apply(config);
    position.getConfigurator().apply(config);

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
    roller.setVoltage(volt);
  }

  public void setPositionVoltage(double volt) {
    position.setVoltage(volt);
  }

  // sets the position of the arm.
  public void setTargetPosition(double position1) { // IMPORTANT -- POSITON1 NOT POSITION
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    position.setControl(m_request.withPosition(position1));
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
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    position.getConfigurator().apply(config);
  }

  // gets the highest possible height of the arm in radians
  public double getMaxPosition() {
    return Constants.IntakeConstants.MAX_POSITION;
  }

  // gets the height of the arm in meters
  public double getPosition() {
    return position.getRotorPosition().getValueAsDouble();
  }

  public boolean isMaxPosition() {
    return Math.abs(getPosition() - getMaxPosition())
        < Constants.IntakeConstants.POSITION_TOLERANCE;
  }

  public double getSpeed() {
    return roller.getRotorPosition().getValueAsDouble();
  }

  public boolean checkIfStalled() {
    return false;
  }
}
