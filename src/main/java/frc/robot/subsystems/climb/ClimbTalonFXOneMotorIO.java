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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimbTalonFXOneMotorIO implements ClimbIO {

  private final TalonFX leftMotor;

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftMotorVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Angle> leftMotorPosition;
  private final StatusSignal<Temperature> leftMotorTemperatureCelsius;

  private Servo servo;
  private double servoPosition = 0.0;

  //   private PIDController positionPIDController = new PIDController(0.1, 0, 0); // CHANGE -
  // CONSTANTS

  private final PositionVoltage leftRequestVoltage;

  private boolean isBraked = true;

  public ClimbTalonFXOneMotorIO(int canIdLeft, int canIdRight) {
    leftMotor = new TalonFX(canIdLeft);

    leftRequestVoltage = new PositionVoltage(0).withSlot(0);
    // Basic Configuration
    TalonFXConfiguration config = Constants.ClimbConstants.CONFIG;

    var slot0Configs = Constants.ClimbConstants.SLOT0CONFIGS;

    leftMotor.getConfigurator().apply(slot0Configs);

    leftMotor.getConfigurator().apply(config);

    // Initialize signals for AdvantageKit
    leftVelocity = leftMotor.getVelocity();
    leftMotorVoltage = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftStatorCurrent = leftMotor.getStatorCurrent();
    leftMotorPosition = leftMotor.getPosition();
    leftMotorTemperatureCelsius = leftMotor.getDeviceTemp();

    // Optimize CAN bus usage by refreshing these signals together
    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.ClimbConstants.FREQUENCY_HZ,
        leftVelocity,
        leftMotorVoltage,
        leftSupplyCurrent,
        leftMotorPosition,
        leftStatorCurrent,
        leftMotorTemperatureCelsius);

    // this the servo
    this.servo = new Servo(Constants.ClimbConstants.SERVO_ID);
  }

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    inputs.motorLeftPosition = leftMotorPosition.getValueAsDouble();

    inputs.motorLeftCurrentSupplyAmps = leftSupplyCurrent.getValueAsDouble();

    inputs.motorLeftCurrentStatorAmps = leftStatorCurrent.getValueAsDouble();

    inputs.motorLeftVolts = leftMotorVoltage.getValueAsDouble();

    inputs.motorLeftSpeed = leftVelocity.getValueAsDouble();

    inputs.isBraked = isBraked;

    inputs.motorLeftTemperatureCelsius = leftMotorTemperatureCelsius.getValueAsDouble();

    inputs.servoPosition = servoPosition;
  }

  @Override
  public void setSpeeds(double leftSpeed, double rightSpeed) {

    leftMotor.setControl(leftRequestVoltage.withVelocity(leftSpeed));
  }

  @Override
  public void setPositions(double leftPosition, double rightPosition) {
    leftMotor.setControl(leftRequestVoltage.withPosition(leftPosition));
  }

  @Override
  public void setServo(double position) {
    this.servoPosition = position;
    servo.setAngle(position);
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
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
  }
}
