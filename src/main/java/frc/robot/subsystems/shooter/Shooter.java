package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // this shouldn't be here but it is for now because we're probably gonna move this
  public static final double TOLERANCE = ShooterConstants.TOLERANCE;

  private DoubleSupplier distanceSupplier;

  // just here for the logging

  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged inputs;

  private final InterpolatingDoubleTreeMap distToSpeedTable;

  private final Notifier logger;

  private double timeSinceLastShotActive;
  private double timeSinceLastShotInactive;

  private String timeSinceLastShotStringActive = "0";
  private String timeSinceLastShotStringInactive = "0";
  // whether or not the shooter automatically charges to
  private boolean automaticallyChargeFully = false;

  /**
   * @param shooterIO the hardware interface
   * @param distanceSupplierMeters the distance supplier for when it goes automatic
   */
  public Shooter(ShooterIO shooterIO, DoubleSupplier distanceSupplierMeters) {
    this.distanceSupplier = distanceSupplierMeters;
    this.shooterIO = shooterIO;
    this.inputs = new ShooterIOInputsAutoLogged();

    this.distToSpeedTable = new InterpolatingDoubleTreeMap();
    // TO DO: populate distToSpeedTable with real valeus

    // comp ones
    // this.speedToTableInit(3.065, 67);
    // this.speedToTableInit(2.60, 60);
    // this.speedToTableInit(3.38, 71);
    // this.speedToTableInit(2.039, 58);

    // this.speedToTableInit(1.63, 57.5);
    // this.speedToTableInit(2.252, 59.5);
    // this.speedToTableInit(2.89, 64.5);
    // this.speedToTableInit(3.5, 68.5);

    // this.speedToTableInit(1.97, 65);
    // this.speedToTableInit(2.61, 71);
    // this.speedToTableInit(3.2, 75);
    // this.speedToTableInit(3.4, 77);
    // this.speedToTableInit(4, 82);

    this.speedToTableInit(1.842, 67);
    this.speedToTableInit(2.162, 69);
    this.speedToTableInit(2.75, 74);
    this.speedToTableInit(3.11, 78);

    this.speedToTableInit(3.453, 84);

    logger =
        new Notifier(
            () -> {
              shooterIO.updateInputs(inputs);
              Logger.processInputs("shooter", inputs);
            });

    logger.startPeriodic(1 / ShooterConstants.FREQUENCY_HZ);

    setAutomaticallyChargeFully(false);

    timeSinceLastShotActive = Timer.getFPGATimestamp();
    timeSinceLastShotInactive = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {}

  // SUBSYSTEM METHODS

  public void setAutomaticSpeed(double scalar) {
    double automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());
    double automaticSpeedScaled = scalar * automaticSpeed;

    shooterIO.setSpeed(automaticSpeedScaled);

    Logger.recordOutput("Shooter/AutomaticSpeedScalar", scalar);
    Logger.recordOutput("Shooter/AutomaticSpeedPreScaled", automaticSpeed);
    Logger.recordOutput("Shooter/AutomaticSpeedScaled", automaticSpeedScaled);
  }

  /**
   * @param speed the speed the flywheel will pid too
   */
  public void setManualSpeed(double speed) {
    shooterIO.setSpeed(speed);
  }

  /**
   * Sets voltage of the shooter, units: Volts(NOT PERCENT OUTPUT)
   *
   * @param voltage
   */
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

  public void setAutomaticallyChargeFully(boolean automaticallyChargeFully) {
    this.automaticallyChargeFully = automaticallyChargeFully;

    Logger.recordOutput("Shooter/AutomaticallyChargeFully", automaticallyChargeFully);
  }

  public boolean isAutomaticallyChargeFully() {
    return automaticallyChargeFully;
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
          setAutomaticSpeed(1);
        },
        this);
  }

  public Command automaticallyChargeWhenNeededRunCommand(
      double automaticPercentage, double speedWhenNotInZone) {
    return new RunCommand(
        () -> {

          // if this flag is in action we're probably gonna shoot soon and should default to full
          // charge
          if (automaticallyChargeFully) {
            setAutomaticSpeed(1);
            return;
          }

          setManualSpeed(speedWhenNotInZone);
        },
        this);
  }

  public Command setAutomaticallyChargeFully(BooleanSupplier automaticallyChargeFully) {
    return new InstantCommand(
        () -> {
          setAutomaticallyChargeFully(automaticallyChargeFully.getAsBoolean());
        });
  }

  private double currentTime = 0;

  public Command shooterRevUpActiveDefaultCommand(BooleanSupplier activePhase) {
    return new RunCommand(
        () -> {
          // first, check if we're in the active phase and (in shooting zone) or (about to finish
          // our cycling time)
          currentTime = Timer.getFPGATimestamp();
          if (activePhase.getAsBoolean()
              && currentTime - timeSinceLastShotActive > ShooterConstants.CYCLE_TIME) {
            setAutomaticSpeed(.5);
          }
        },
        this);
  }

  public void resetTimeSinceLastShotActive() {
    timeSinceLastShotActive = Timer.getFPGATimestamp();
    timeSinceLastShotStringActive = timeSinceLastShotStringActive + ", " + timeSinceLastShotActive;
    Logger.recordOutput("CycleData/timeSinceLastShotStringActive", timeSinceLastShotStringActive);
  }

  public void resetTimeSinceLastShotInactive() {
    timeSinceLastShotInactive = Timer.getFPGATimestamp();
    timeSinceLastShotStringInactive =
        timeSinceLastShotStringInactive + ", " + timeSinceLastShotInactive;
    Logger.recordOutput(
        "CycleData/timeSinceLastShotStringInactive", timeSinceLastShotStringInactive);
  }

  public Command resetTimeSinceLastShotCommand(boolean activePeriod) {
    return new InstantCommand(
        () -> {
          if (activePeriod) {
            resetTimeSinceLastShotActive();
          } else {
            resetTimeSinceLastShotInactive();
          }
        });
  }
  // HELPER METHODS
  /**
   * uses InterpolatingDoubleTreeMap to deduce speed to output as a function of distance
   *
   * @param distance dist from goal to shoot into
   * @return
   */
  private double getSpeedFromDistance(double distance) {

    // dummy value for now
    // return distance;

    // commented otu for now becaues this is flat since we don't have values, I want something we
    // can debug from
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
