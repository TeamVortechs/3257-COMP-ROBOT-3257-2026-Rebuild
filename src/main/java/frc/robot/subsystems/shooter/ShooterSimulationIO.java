package frc.robot.subsystems.shooter;

// import edu.wpi.first.math.controller.ElevatorFeedforward;

// things i think i should import from ShooterRotationManager.java
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
// imported from ArmSimulationIO.java
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


//get constnts

public class ShooterSimulationIO implements ShooterIO {
    // the motor that spins the things to shoot balls?
    private final DCMotorSim rollerMotorsSim;
    private final double targetSpeed = 0;
    private final boolean isOnTarget = false;
    public ShooterSimulationIO(){
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

        inputs.targetSpeed = targetSpeed;
        inputs.isOnTarget = isOnTarget;

        rollerMotorsSim.update(0.02);
  }

    @Override
    public void setSpeed(double speed) {
        // no clue if this works... but it's in last years arm code, so i used the set voltage

        // rollerMotorsSim.setInputVoltage(speed * 12); // lol i hope that works
        setVoltage(speed*12);
    }

    @Override
    public void setVoltage(double voltage) {
        rollerMotorsSim.setInputVoltage(voltage);
    }

    // might as well include ts W tech
    public void stop(){
        setVoltage(0);
    }

    public double getSpeed(){
        return rollerMotorsSim.getAngularVelocityRPM();
    }

    @Override
    public boolean isOnTarget(){
        return targetSpeed == getSpeed();
    }
}
