package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
    motionMagicConfigs.MotionMagicCruiseVelocity = 3;
    motionMagicConfigs.MotionMagicAcceleration = 2.5;
    motionMagicConfigs.MotionMagicJerk = 10;

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
        50, rollerVelocity, rollerMotorVoltage, rollerSupplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionVelocity, positionMotorVoltage, positionSupplyCurrent);
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
    return 0;
  }

  public double getVoltage() {
    return 0;
  }

  // setters for motors
  public void set(double volt) {
    roller.setVoltage(volt);
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

  // rebuilds the pid constants of the motors
  public void rebuildMotorsPID() {}

  /** Stops the motor immediately */
  public void stop() {}

  public void resetEncoders() {}

  public void setBraked(boolean braked) {}

  // gets the highest possible height of the arm in radians
  public double getMaxPosition() {
    return 0;
  }

  // gets the height of the arm in meters
  public double getPosition() {
    return position.getRotorPosition().getValueAsDouble();
  }

  public boolean isMaxPosition() {
    return false;
  }

  public double getSpeed() {
    return roller.getRotorPosition().getValueAsDouble();
  }

  public boolean checkIfStalled() {
    return false;
  }
}
