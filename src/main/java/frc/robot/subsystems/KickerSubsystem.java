package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  TalonFX kicker;
  /** Creates a new ExampleSubsystem. */
  public KickerSubsystem(TalonFX kicker) {
    this.kicker = kicker;
    TalonFXConfigurator configs = kicker.getConfigurator();
    OpenLoopRampsConfigs loopConfigs =
        new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(2.0);
    MotorOutputConfigs outputConfigs =
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    configs.apply(loopConfigs);
    configs.apply(outputConfigs);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command kickCmd(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runEnd(
        () -> {
          kicker.set(speed);
        },
        () -> {
          kicker.set(0);
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
