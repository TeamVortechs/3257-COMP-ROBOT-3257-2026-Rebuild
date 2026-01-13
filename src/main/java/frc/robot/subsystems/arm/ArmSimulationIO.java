package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ArmSimulationIO implements ArmIO {
  private final DCMotorSim armMotorsSim;
  private final DCMotorSim rollerMotorsSim;

  private PIDController armPIDController = new PIDController(0.9, 0, 0.1);
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);

  private double targetVel = 1;

  private double distance = 0;

  public ArmSimulationIO() {
    this.armMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
    this.rollerMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
  }

  // sets the PID target angle
  @Override
  public void PIDVoltage(double targetAngle) {
    double currentAngle = armMotorsSim.getAngularPositionRotations();
    double inputVoltage =
        armPIDController.calculate(currentAngle, targetAngle)
            + armFeedforward.calculate(targetAngle * Math.PI * 2 - (Math.PI / 2), targetVel);
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    setVoltage(inputVoltage);
    // System.out.println("Voltage being sent in PID Voltage");
  }

  // advantage kti logging stuff
  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    inputs.armMotor1CurrentHeightMeter = armMotorsSim.getAngularPositionRad();
    inputs.armMotor1CurrentSpeedMeter = armMotorsSim.getAngularVelocityRadPerSec();

    inputs.armMotor1CurrentAmps = armMotorsSim.getCurrentDrawAmps();
    inputs.armMotor1AppliedVolts = armMotorsSim.getInputVoltage();

    inputs.armMotor2CurrentAmps = 0;
    inputs.armMotor2AppliedVolts = 0;

    inputs.armMotor2CurrentSpeedMeter = 0;
    inputs.armMotor2CurrentHeightMeter = 0;

    inputs.isStalled = false;

    inputs.rollerAmps = rollerMotorsSim.getCurrentDrawAmps();
    inputs.rollerVolts = rollerMotorsSim.getInputVoltage();
    inputs.rollerSpeed = rollerMotorsSim.getAngularVelocityRPM();

    inputs.canRangeDistance = distance;

    armMotorsSim.update(0.02);
    rollerMotorsSim.update(0.02);
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerMotorsSim.setInputVoltage(speed * 12); // lol i hope that works
  }

  // no extra stuff here, just stop the motor
  @Override
  public void stop() {
    setVoltage(0);
  }

  // this is sim so kinda gotta estimate
  @Override
  public double getMaxAngle() {
    return 100;
  }

  @Override
  public void setVoltage(double volt) {
    armMotorsSim.setInputVoltage(MathUtil.clamp(volt, -12, 12));
  }

  @Override
  public void resetEncoders() {
    armMotorsSim.setAngle(0);
  }

  @Override
  public double getAngle() {
    return armMotorsSim.getAngularPositionRotations();
  }

  @Override
  public boolean isMaxAngle() {
    return Math.abs(getMaxAngle() - getAngle()) > 0.1;
  }

  @Override
  public double getDistance() {
    return distance;
  }

  @Override
  public double getRollerSpeed() {
    // TODO Auto-generated method stub
    return rollerMotorsSim.getAngularAccelerationRadPerSecSq();
  }

  @Override
  public void setCanrangeDistance(double dist) {
    this.distance = dist;
  }
}
