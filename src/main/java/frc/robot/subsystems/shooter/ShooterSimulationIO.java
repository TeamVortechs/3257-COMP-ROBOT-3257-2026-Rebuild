package frc.robot.subsystems.shooter;

// import edu.wpi.first.math.controller.ElevatorFeedforward;

// things i think i should import from ShooterRotationManager.java
// imported from ArmSimulationIO.java
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// get constnts

public class ShooterSimulationIO implements ShooterIO {
  // the motor that spins the things to shoot balls?
  private final DCMotorSim rollerMotorsSim;
  private double targetSpeed = 0;
  private boolean isOnTarget = false;

  public ShooterSimulationIO() {
    // now this motor exists in advantage kit?
    this.rollerMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
  }

  // update inputs on roller motors
  @Override
  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    inputs.amps = rollerMotorsSim.getCurrentDrawAmps();
    inputs.voltage = rollerMotorsSim.getInputVoltage();
    inputs.speed = rollerMotorsSim.getAngularVelocityRPM();

    isOnTarget = isOnTarget();

    inputs.targetSpeed = targetSpeed;
    inputs.isOnTarget = isOnTarget;

    rollerMotorsSim.update(0.02);
  }

  @Override
  public void setSpeed(double speed) {
    // no clue if this works... but it's in last years arm code, so i used the set voltage

    // rollerMotorsSim.setInputVoltage(speed * 12); // lol i hope that works
    setVoltage(speed / 502.747);

    targetSpeed = speed;
  }

  @Override
  public void setVoltage(double voltage) {
    rollerMotorsSim.setInputVoltage(voltage);
  }

  // might as well include ts W tech
  public void stop() {
    setVoltage(0);
  }

  public double getSpeed() {
    return rollerMotorsSim.getAngularVelocityRPM();
  }

  @Override
  public boolean isOnTarget() {
    return Math.abs((getSpeed() - targetSpeed)) <= 0.5;
  }
}
