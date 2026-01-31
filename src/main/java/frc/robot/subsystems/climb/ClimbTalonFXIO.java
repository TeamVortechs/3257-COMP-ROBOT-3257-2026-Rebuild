package frc.robot.subsystems.climb;

// import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimbTalonFXIO implements ClimbIO {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftMotorVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Angle> leftMotorPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightMotorVoltage;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Angle> rightMotorPosition;
  private Servo servo;
  private double servoPosition = 0.0;

  //   private PIDController positionPIDController = new PIDController(0.1, 0, 0); // CHANGE -
  // CONSTANTS

  private boolean manual = true;

  private double targetLeftPos = 0;
  private double targetRightPos = 0;

  public ClimbTalonFXIO(int canIdLeft, int canIdRight) {
    leftMotor = new TalonFX(canIdLeft);
    rightMotor = new TalonFX(canIdRight);

    // Basic Configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.ClimbConstants.CURRENT_LIMIT; // Prevent breaker trips
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = Constants.ClimbConstants.kS; // Add 0.1 V output to overcome static friction
    slot0Configs.kV =
        Constants.ClimbConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = Constants.ClimbConstants.kP; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = Constants.ClimbConstants.kI; // no output for integrated error
    slot0Configs.kD = Constants.ClimbConstants.kD; // no output for error derivative

    leftMotor.getConfigurator().apply(slot0Configs);

    leftMotor.getConfigurator().apply(config);

    rightMotor.getConfigurator().apply(slot0Configs);

    rightMotor.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    leftVelocity = leftMotor.getVelocity();
    leftMotorVoltage = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftMotorPosition = leftMotor.getPosition();

    rightVelocity = rightMotor.getVelocity();
    rightMotorVoltage = rightMotor.getMotorVoltage();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightMotorPosition = rightMotor.getPosition();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.FREQUENCY_HZ,
        leftVelocity,
        leftMotorVoltage,
        leftSupplyCurrent,
        leftMotorPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.FREQUENCY_HZ,
        rightVelocity,
        rightMotorVoltage,
        rightSupplyCurrent,
        rightMotorPosition);

    // this the servo
    this.servo = new Servo(Constants.ClimbConstants.ID);
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    inputs.motorLeftPosition = leftMotorPosition.getValueAsDouble();
    inputs.motorRightPosition = rightMotorPosition.getValueAsDouble();

    inputs.motorLeftAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.motorRightAmps = rightSupplyCurrent.getValueAsDouble();

    inputs.motorLeftVolts = leftMotorVoltage.getValueAsDouble();
    inputs.motorRightVolts = rightMotorVoltage.getValueAsDouble();

    inputs.motorLeftSpeed = leftVelocity.getValueAsDouble();
    inputs.motorRightSpeed = rightVelocity.getValueAsDouble();

    inputs.servoPosition = servoPosition;

    if (!manual) {
      final PositionVoltage leftRequest = new PositionVoltage(0).withSlot(0);
      // motor.set(speed);
      leftMotor.setControl(leftRequest.withPosition(targetLeftPos));

      final PositionVoltage rightRequest = new PositionVoltage(0).withSlot(0);
      // motor.set(speed);
      leftMotor.setControl(rightRequest.withPosition(targetRightPos));
      //
      // leftMotor.setVoltage(positionPIDController.calculate(leftMotorPosition.getValueAsDouble(),
      // targetLeftPos)); // CHANGE MAYBE
      //
      // rightMotor.setVoltage(positionPIDController.calculate(rightMotorPosition.getValueAsDouble(), targetRightPos));
    }
  }

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    final VelocityVoltage leftRequest = new VelocityVoltage(0).withSlot(0);
    // motor.set(speed);
    leftMotor.setControl(leftRequest.withVelocity(leftSpeed));

    final VelocityVoltage rightRequest = new VelocityVoltage(0).withSlot(0);
    // motor.set(speed);
    rightMotor.setControl(rightRequest.withVelocity(rightSpeed));
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
  public void setServo(double position) {
    this.servoPosition = position;
    servo.setAngle(position);
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
  }
}
