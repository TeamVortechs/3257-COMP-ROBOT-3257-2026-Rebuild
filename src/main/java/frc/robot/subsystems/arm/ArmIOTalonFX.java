package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class ArmIOTalonFX implements ArmIO {

  private TalonFX arm;

  public ArmIOTalonFX(int armID, String canbusName) {
    arm = new TalonFX(armID, canbusName);

    TalonFXConfiguration armMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(Constants.ARM_CURRENT_LIMIT))
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    var slot0Configs = armMotorConfigs.Slot0;
    slot0Configs.kS = Constants.ARM_kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = Constants.ARM_kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = Constants.ARM_kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = Constants.ARM_kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = Constants.ARM_kI; // no output for integrated error
    slot0Configs.kD = Constants.ARM_kD; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.kG = Constants.ARM_kG; // account for the weight of the arm
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

    var motionMagicConfigs = armMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ARM_SPEED_LIMIT;
    motionMagicConfigs.MotionMagicAcceleration = Constants.ARM_ACCELERATION_LIMIT;
    motionMagicConfigs.MotionMagicJerk = Constants.ARM_JERK_LIMIT;

    arm.getConfigurator().apply(armMotorConfigs);

    // Set motor to Brake mode by default.
    arm.setNeutralMode(NeutralModeValue.Brake);
  }

  // sets the PID target angle
  @Override
  public void PIDVoltage(double targetAngle) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    arm.setControl(m_request.withPosition(targetAngle));
    // System.out.println("Voltage being sent in PID Voltage");
  }

  // advantage kti logging stuff
  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    inputs.armMotor1CurrentHeightMeter = getAngle();
    inputs.armMotor1CurrentSpeedMeter = getRollerSpeed();

    inputs.armMotor1CurrentAmps = getCurrent();
    inputs.armMotor1AppliedVolts = getVoltage();

    inputs.armMotor2CurrentAmps = 0;
    inputs.armMotor2AppliedVolts = 0;

    inputs.armMotor2CurrentSpeedMeter = 0;
    inputs.armMotor2CurrentHeightMeter = 0;

    inputs.isStalled = false; // ask about this
  }

  // no extra stuff here, just stop the motor
  @Override
  public void stop() {
    setVoltage(0);
  }

  // this is sim so kinda gotta estimate
  @Override
  public double getMaxAngle() {
    return Constants.ARM_MAX_ANGLE;
  }

  @Override
  public void setVoltage(double volt) {
    arm.setVoltage(volt);
  }

  @Override
  public void resetEncoders() {
    arm.setPosition(0);
  }

  @Override
  public double getAngle() {
    return arm.getPosition().getValueAsDouble();
  }

  @Override
  public boolean isMaxAngle() {
    return Math.abs(getMaxAngle() - getAngle()) > Constants.ARM_TOLERANCE;
  }
}
