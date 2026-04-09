package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // this shouldn't be here but it is for now because we're probably gonna move this
  public static final double TOLERANCE = ShooterConstants.TOLERANCE;

  private DoubleSupplier distanceSupplier;

  // just here for the logging

  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged inputs;

  // TO DO: add values to table
  private final InterpolatingDoubleTreeMap distToSpeedTable;

  // private final Notifier logger;

  // wether or not the shooter automatically charges to

  /**
   * @param shooterIO the hardware interface
   * @param distanceSupplierMeters the distance supplier for when it goes automatic
   */
  public Shooter(ShooterIO shooterIO, DoubleSupplier distanceSupplierMeters) {
    this.distanceSupplier = distanceSupplierMeters;
    this.shooterIO = shooterIO;
    this.inputs = new ShooterIOInputsAutoLogged();

    this.distToSpeedTable = new InterpolatingDoubleTreeMap();

    // this.speedToTableInit(2.155, 56);
    // this.speedToTableInit(2.573, 60);
    // this.speedToTableInit(3.168, 65.75);
    // this.speedToTableInit(4.13, 71);

    this.speedToTableInit(1.753, 61);
    this.speedToTableInit(1.91, 62);
    // this.speedToTableInit(2.25, 63);
    this.speedToTableInit(2.155, 63.85964912);
    // this.speedToTableInit(2.155, 63.85964912);
    this.speedToTableInit(2.573, 68.42105263);
    this.speedToTableInit(3.299, 76.4);
    // this.speedToTableInit(4, 77);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("shooter", inputs);
  }

  // SUBSYSTEM METHODS

  public void setAutomaticSpeed() {
    double automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());

    shooterIO.setSpeed(automaticSpeed);
  }

  /**
   * @param speed the speed the flywheel will pid too
   */
  public void setManualSpeed(double speed) {
    shooterIO.setSpeed(speed);
  }

  public void setVoltage(double voltage) {
    shooterIO.setVoltage(voltage);
  }

  public double getSpeed() {
    return shooterIO.getSpeed();
  }

  /**
   * @return wether the speed is the target speed
   */
  public boolean isOnTarget() {
    return shooterIO.isOnTargetSpeed();
  }

  // COMMANDS
  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setManualSpeedCommand(double speed) {
    return new InstantCommand(() -> this.setManualSpeed(speed), this);
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setManualSpeedRunCommand(double speed) {
    return Commands.startRun(
        () -> {
          this.setManualSpeed(speed);
        },
        () -> {},
        this);
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setManualSpeedRunCommand(DoubleSupplier speed) {
    return Commands.startRun(
        () -> {
          this.setManualSpeed(speed.getAsDouble());
        },
        () -> {},
        this);
  }

  public Command setVoltageRunCommand(double voltage) {
    return Commands.startRun(() -> this.setVoltage(voltage), () -> {}, this);
  }

  /**
   * returns a command that revs up to shoot at the distance and ends immediately
   *
   * @return
   */
  public Command setAutomaticCommandRun() {
    return new RunCommand(
        () -> {
          setAutomaticSpeed();
        },
        this);
  }

  // HELPER METHODS
  /**
   * uses InterpolatingDoubleTreeMap to deduce speed to output as a function of distance
   *
   * @param distance dist from goal to shoot into
   * @return
   */
  private double getSpeedFromDistance(double distance) {
    return this.distToSpeedTable.get(distance);
  }

  /**
   * inserts values into the distToSpeedTable
   *
   * @param distance dist from goal to shoot into
   * @param speed speed for shooter to be at
   * @return
   */
  private void speedToTableInit(double distance, double speed) {
    this.distToSpeedTable.put(distance, speed);
  }

  // the constants here should probably be more and move but that's later when this is transferred
  // to the right project
  // add this to the robot class or this won't work: SignalLogger.setPath("/media/sda1/");
  /**
   * Gets the system identification routine for this specific subsystem
   *
   * @return the sysid routine
   */
  public SysIdRoutine BuildSysIdRoutine() {

    SysIdRoutine m_SysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(ShooterConstants.RAMP_RATE_VOLTS_SYSID)
                    .per(Seconds), // Ramp Rate in Volts / Seconds
                Volts.of(ShooterConstants.DYNAMIC_STEP_VOLTS_SYSID), // Dynamic Step Voltage
                null, // Use default timeout (10 s)
                (state) ->
                    SignalLogger.writeString(
                        "state", state.toString()) // Log state with Phoenix SignalLogger class
                ),
            new SysIdRoutine.Mechanism(
                (volts) -> shooterIO.setVoltage(volts.in(Volts)), null, this));
    return m_SysIdRoutine;
  }
}
