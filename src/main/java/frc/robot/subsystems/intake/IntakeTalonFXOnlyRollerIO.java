package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
// CHANGE PID VALUES !!!!

// only uses the roller motor for the prototype robot without positon motor. This is not meant to be
// used on the final
// robot
public class IntakeTalonFXOnlyRollerIO implements IntakeIO {

  private final TalonFX roller;

  // StatusSignals allow for high-frequency, synchronous data collection
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerMotorVoltage;
  private final StatusSignal<Current> rollerStatorCurrent;
  private final StatusSignal<Current> rollerSupplyCurrent;

  private final StatusSignal<Temperature> rollerTemperatureCelsius;

  private boolean isBrakedRoller = true;

  public IntakeTalonFXOnlyRollerIO(int canIdRoller, int canIdPosition) {
    roller = new TalonFX(canIdRoller);

    // Basic Configuration
    TalonFXConfiguration config = Constants.IntakeConstants.ROLLER_CONFIG;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration =
        Constants.IntakeConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.MOTION_MAGIC_JERK;

    roller.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    rollerVelocity = roller.getVelocity();
    rollerMotorVoltage = roller.getMotorVoltage();
    rollerStatorCurrent = roller.getStatorCurrent();
    rollerSupplyCurrent = roller.getSupplyCurrent();
    rollerTemperatureCelsius = roller.getDeviceTemp();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.IntakeConstants.FREQUENCY_HZ,
        rollerVelocity,
        rollerMotorVoltage,
        rollerStatorCurrent,
        rollerSupplyCurrent,
        rollerTemperatureCelsius);
  }

  // updates the given inputs with new values(advantage kit stuff)
  public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.rollerAmpsStator = rollerStatorCurrent.getValueAsDouble();
    inputsAutoLogged.rollerAmpsSupply = rollerSupplyCurrent.getValueAsDouble();
    inputsAutoLogged.rollerVolts = rollerMotorVoltage.getValueAsDouble();
    inputsAutoLogged.rollerSpeed = rollerVelocity.getValueAsDouble();
    inputsAutoLogged.rollerTemperatureCelsius = rollerTemperatureCelsius.getValueAsDouble();

    inputsAutoLogged.isBrakedRoller = isBrakedRoller;
  }

  // getters for motors

  public double getCurrent() {
    return rollerStatorCurrent.getValueAsDouble();
  }

  public double getVoltage() {
    return rollerMotorVoltage.getValueAsDouble();
  }

  // setters for motors
  public void setRollerVoltage(double volt) {
    roller.setControl(new VoltageOut(volt));
  }

  public void setPositionVoltage(double volt) {}

  public void setPositionControl(double position1) { // IMPORTANT - POSITON1 NOT POSITION
  }

  public double getTargetPosition() {
    return 0;
  }
  // misc methods

  public void resetEncoders() {}

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

  public void setBrakedPosition(boolean braked) {}

  // gets the highest possible height of the arm in radians
  public double getMaxPosition() {
    return Constants.IntakeConstants.MAX_POSITION;
  }

  /**
   * @return gets the position of the arm in radians
   */
  public double getPosition() {
    return 0;
  }

  public boolean isMaxPosition() {
    return true;
  }

  public double getSpeed() {
    return roller.getVelocity().getValueAsDouble();
  }

  public boolean checkIfStalled() {
    return (roller.getMotorVoltage().getValueAsDouble()
        > Constants.IntakeConstants.ROLLER_STALLED_VOLTS);
  }
}
