package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeSimulationIO {


  private final DCMotorSim rollerMotorsSim;
  private final DCMotorSim positionMotorsSim;

  private double targetPosition = 0;

  private PIDController positionPIDController = new PIDController(0.9, 0, 0.1);


    
  public IntakeSimulationIO() {
    // now this motor exists in advantage kit?
    this.rollerMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));

    this.positionMotorsSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
        DCMotor.getKrakenX60(1));
  }

    // updates the given inputs with new values(advantage kit stuff)
  public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {
    inputsAutoLogged.rollerAmps = rollerMotorsSim.getCurrentDrawAmps();
    inputsAutoLogged.rollerVolts = rollerMotorsSim.getInputVoltage();
    inputsAutoLogged.rollerSpeed = rollerMotorsSim.getAngularVelocityRadPerSec();

    inputsAutoLogged.positionAmps = positionMotorsSim.getCurrentDrawAmps();
    inputsAutoLogged.positionVolts = positionMotorsSim.getInputVoltage();
    inputsAutoLogged.positionSpeed = positionMotorsSim.getAngularVelocityRadPerSec();

    inputsAutoLogged.position = positionMotorsSim.getAngularPositionRotations();
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
    rollerMotorsSim.setInputVoltage(volt);
  }

  // sets the position of the arm. 
  public void setTargetPosition(double position) {
    double currentAngle = positionMotorsSim.getAngularPositionRotations();
    double inputVoltage =
        positionPIDController.calculate(currentAngle, position);
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    positionMotorsSim.setInputVoltage(inputVoltage);
    // System.out.println("Voltage being sent in PID Voltage");
  }

  public double getTargetPosition() {
    return 0;
  }
  // misc methods

  // rebuilds the pid constants of the motors
  public void rebuildMotorsPID() {}

  /** Stops the motor immediately */
  public void stop() {}
  ;

  public void resetEncoders() {}

  public void setBraked(boolean braked) {}

  // gets the highest possible height of the arm in radians
  public double getMaxPosition() {
    return 0;
  }

  // gets the height of the arm in meters
  public double getPosition() {
    return 0;
  }

  public boolean isMaxPosition() {
    return false;
  }
  public double getSpeed() {
    return 0;
  }

  public boolean checkIfStalled() {
    return false;
  }
}
