package frc.robot.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX shooter;
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(TalonFX shooter) {
    this.shooter = shooter;
    TalonFXConfigurator configs = shooter.getConfigurator();
    OpenLoopRampsConfigs loopConfigs =
        new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(3.0);

    configs.apply(loopConfigs);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shootCmd(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runEnd(
        () -> {
          shooter.set(speed);
        },
        () -> {
          shooter.set(0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
