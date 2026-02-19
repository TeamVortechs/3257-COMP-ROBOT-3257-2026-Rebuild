package frc.robot.subsystems.climb;

// import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClimbTalonFXIO implements ClimbIO {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftMotorVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Angle> leftMotorPosition;
  private final StatusSignal<Temperature> leftMotorTemperatureCelsius;

  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightMotorVoltage;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Angle> rightMotorPosition;
  private final StatusSignal<Temperature> rightMotorTemperatureCelsius;

  //   private PIDController positionPIDController = new PIDController(0.1, 0, 0); // CHANGE -
  // CONSTANTS

  private boolean manual = true;

  private double targetLeftPos = 0;
  private double targetRightPos = 0;

  private final PositionVoltage leftRequestVoltage;
  private final PositionVoltage rightRequestVoltage;

  private boolean isBraked = true;

  public ClimbTalonFXIO(int canIdLeft, int canIdRight) {
    leftMotor = new TalonFX(canIdLeft);
    rightMotor = new TalonFX(canIdRight);

    leftRequestVoltage = new PositionVoltage(0).withSlot(0);
    rightRequestVoltage = new PositionVoltage(0).withSlot(0);
    // Basic Configuration
    TalonFXConfiguration config = Constants.ClimbConstants.CONFIG;

    var slot0Configs = Constants.ClimbConstants.SLOT0CONFIGS;

    leftMotor.getConfigurator().apply(slot0Configs);

    leftMotor.getConfigurator().apply(config);

    rightMotor.getConfigurator().apply(slot0Configs);

    rightMotor.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    leftVelocity = leftMotor.getVelocity();
    leftMotorVoltage = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftStatorCurrent = leftMotor.getStatorCurrent();
    leftMotorPosition = leftMotor.getPosition();
    leftMotorTemperatureCelsius = leftMotor.getDeviceTemp();

    rightVelocity = rightMotor.getVelocity();
    rightMotorVoltage = rightMotor.getMotorVoltage();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightStatorCurrent = rightMotor.getStatorCurrent();
    rightMotorPosition = rightMotor.getPosition();
    rightMotorTemperatureCelsius = rightMotor.getDeviceTemp();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.ClimbConstants.FREQUENCY_HZ,
        leftVelocity,
        leftMotorVoltage,
        leftSupplyCurrent,
        leftMotorPosition,
        leftStatorCurrent,
        leftMotorTemperatureCelsius);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.ClimbConstants.FREQUENCY_HZ,
        rightVelocity,
        rightMotorVoltage,
        rightSupplyCurrent,
        rightMotorPosition,
        rightStatorCurrent,
        rightMotorTemperatureCelsius);

    // this the servo
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    inputs.motorLeftPosition = leftMotorPosition.getValueAsDouble();
    inputs.motorRightPosition = rightMotorPosition.getValueAsDouble();

    inputs.motorLeftCurrentSupplyAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.motorRightCurrentSupplyAmps = rightSupplyCurrent.getValueAsDouble();

    inputs.motorLeftCurrentStatorAmps = leftStatorCurrent.getValueAsDouble();
    inputs.motorRightCurrentStatorAmps = rightStatorCurrent.getValueAsDouble();

    inputs.motorLeftVolts = leftMotorVoltage.getValueAsDouble();
    inputs.motorRightVolts = rightMotorVoltage.getValueAsDouble();

    inputs.motorLeftSpeed = leftVelocity.getValueAsDouble();
    inputs.motorRightSpeed = rightVelocity.getValueAsDouble();

    inputs.isBraked = isBraked;

    inputs.motorLeftTemperatureCelsius = leftMotorTemperatureCelsius.getValueAsDouble();
    inputs.motorRightTemperatureCelsius = rightMotorTemperatureCelsius.getValueAsDouble();

    if (!manual) {
      // motor.set(speed);
      leftMotor.setControl(leftRequestVoltage.withPosition(targetLeftPos));
      // motor.set(speed);
      leftMotor.setControl(rightRequestVoltage.withPosition(targetRightPos));
      //
      // leftMotor.setVoltage(positionPIDController.calculate(leftMotorPosition.getValueAsDouble(),
      // targetLeftPos)); // CHANGE MAYBE
      //
      // rightMotor.setVoltage(positionPIDController.calculate(rightMotorPosition.getValueAsDouble(), targetRightPos));
    }
  }

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    // motor.set(speed);
    leftMotor.setControl(leftRequestVoltage.withVelocity(leftSpeed));
    // motor.set(speed);
    rightMotor.setControl(rightRequestVoltage.withVelocity(rightSpeed));
    // leftMotor.setVoltage(leftSpeed / 502.747);
    // rightMotor.setVoltage(rightSpeed / 502.747);

    manual = true;
  }

  @Override
  public void setPositions(double leftPosition, double rightPosition) {
    manual = false;
    targetLeftPos = leftPosition;
    targetRightPos = rightPosition;
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
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

    leftMotor.setNeutralMode(neutralModeValue);
    rightMotor.setNeutralMode(neutralModeValue);
  }
}
