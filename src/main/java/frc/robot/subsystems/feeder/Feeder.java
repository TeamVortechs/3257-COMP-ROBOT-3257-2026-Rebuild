package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FeederConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private FeederIO feederIO;
  private FeederIOInputsAutoLogged inputs;

  private BooleanSupplier shooterOnVelocity;
  private BooleanSupplier driveIsValid;

  // I was advised not to add this by jason but john said we should. For now I'll add it so we don't
  // have to add it later. We can just supply true in the constructor
  private BooleanSupplier visionSeesTag;
  private Timer debounceTimer;

  private final Notifier hardwareLogger;
  private final Notifier validityLogger;

  // here so we can log it
  // caching this value and calculating it in periodic so we can log it for the driver
  /**
   * @param feederIO the hardware interface
   */
  public Feeder(
      FeederIO feederIO,
      BooleanSupplier driveIsValid,
      BooleanSupplier shooterOnVelocity,
      BooleanSupplier visionSeesTag) {
    this.feederIO = feederIO;
    this.inputs = new FeederIOInputsAutoLogged();

    this.driveIsValid = driveIsValid;
    this.shooterOnVelocity = shooterOnVelocity;
    this.visionSeesTag = visionSeesTag;

    debounceTimer = new Timer();

    hardwareLogger =
        new Notifier(
            () -> {
              feederIO.updateInputs(inputs);
              Logger.processInputs("feeder", inputs);
            });

    validityLogger =
        new Notifier(
            () -> {
              calculateValidityToFeed();
            });

    hardwareLogger.startPeriodic(1 / FeederConstants.SUBSYSTEM_LOGGING_FREQUENCY_HERTZ);
    validityLogger.startPeriodic(1 / FeederConstants.VALIDITY_LOGGING_FREQUENCY_HERTZ);
  }

  @Override
  public void periodic() {}

  // SUBSYSTEM METHODS

  /**
   * @param speed the percentage of battery voltage the belts will take. -1 to 1
   */
  public void setPercentMotorOutput(double speed) {
    feederIO.setPercentMotorOutput(speed);
  }

  /**
   * @return speed in Rotations per second
   */
  public double getSpeed() {
    return feederIO.getSpeed();
  }

  // HELPER METHODS
  private boolean calculateValidityToFeed() {
    // this stuff should wait on debounce, probably not shooter on speed though
    if (!driveIsValid.getAsBoolean() || !visionSeesTag.getAsBoolean()) {
      debounceTimer.restart();
    }

    // logging part:

    boolean isValid =
        debounceTimer.hasElapsed(FeederConstants.VALIDITY_DEBOUNCE_TIME_SEC)
            && shooterOnVelocity.getAsBoolean();

    Logger.recordOutput("Feeder/IsValidToFeed", isValid);
    Logger.recordOutput("DebounceTime", debounceTimer.get());

    return isValid;
  }

  // COMMANDS
  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param percentage the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorCommand(double percentage) {
    return new InstantCommand(() -> this.setPercentMotorOutput(percentage));
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param percentage the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorRunCommand(double percentage) {
    return Commands.run(() -> this.setPercentMotorOutput(percentage), this);
  }

  /**
   * feeds when the drive, limelight, and shooter are ontarget
   *
   * @param percentage the percentage of battery to supply to the feeder
   * @return the command
   */
  public Command feedWhenValidRunCommand(double percentage) {
    return Commands.run(
        () -> {
          if (calculateValidityToFeed()) {
            this.setPercentMotorOutput(percentage);
          } else {
            this.setPercentMotorOutput(0);
          }
        },
        this);
  }

  /** can swap this for the other command in auto */
  public Command feedWhenShooterIsRevvedCommand(double percentage) {
    return Commands.run(
        () -> {
          if (shooterOnVelocity.getAsBoolean()) {
            this.setPercentMotorOutput(percentage);
          } else {
            this.setPercentMotorOutput(0);
          }
        },
        this);
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
                Volts.of(FeederConstants.RAMP_RATE_VOLTS_SYSID)
                    .per(Seconds), // Ramp Rate in Volts / Seconds
                Volts.of(FeederConstants.DYNAMIC_STEP_VOLTS_SYSID), // Dynamic Step Voltage
                null, // Use default timeout (10 s)
                (state) ->
                    SignalLogger.writeString(
                        "state", state.toString()) // Log state with Phoenix SignalLogger class
                ),
            new SysIdRoutine.Mechanism(
                (volts) -> feederIO.setVoltage(volts.in(Volts)), null, this));
    return m_SysIdRoutine;
  }
}

// IGNORE THIS!!!

/*
 * two modes: supply a distance and have it wind up to that or
 * supply hand values and have it wind up to that
 *
 */

 /* sysid routine:
 *   // To-do: Move sysId settings to the constants file
 public SysIdRoutine BuildSysIdRoutine()
 {
   this.m_SysIdRoutine = new SysIdRoutine(
     new SysIdRoutine.Config(
        Volts.of(0.25).per(Seconds),  // Ramp Rate in Volts / Seconds
        Volts.of(1), // Dynamic Step Voltage
        null,          // Use default timeout (10 s)
        (state) -> SignalLogger.writeString("state", state.toString()) // Log state with Phoenix SignalLogger class
     ),
     new SysIdRoutine.Mechanism(
        (volts) -> m_ElevatorMotor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
     )
  );
  return this.m_SysIdRoutine;
 }


 want to slow ramp on elevator because it is has a fast hardstop

 can use sysid on drivetrain
 must all be done in one enable



   private void configureElevatorDebugBindings()
 {
   Elevator elevator = m_Manager.getSubsystemOfType(Elevator.class).get();
   SmartDashboard.putData(elevator);

   CommandSwerveDrivetrain drivetrain = m_Manager.getSubsystemOfType(CommandSwerveDrivetrain.class).get();

   SysIdRoutine sysIdRoutine = elevator.BuildSysIdRoutine();

   controller.a().onTrue(elevator.MoveToLevel(HEIGHTS.ONE));
   controller.b().onTrue(elevator.MoveToLevel(HEIGHTS.TWO));
   controller.y().onTrue(elevator.MoveToLevel(HEIGHTS.THREE));
   controller.x().onTrue(elevator.MoveToLevel(HEIGHTS.FOUR));

   controller.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
   controller.povRight().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
   controller.povDown().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
   controller.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));

   System.out.println("[Wolfpack] Elevator Debug bindings successfully configured.");
 }


     SignalLogger.setPath("/media/sda1/");


     take log -> convert to wpilog -> put in sysid -> select correct motor(the motor you ran it on) -> expand logs on that motor -> look for log entry called state -> pull velocity/position measurements depending on wether you are doing velocity or position measurements


     look over at the displacement from the previous frame to the current frame. We wanna see if it's the filter or not

     plot framerate over time


     can get away with one camera for just

     get away with as many cameras as you need to get full 360 degree coverage

     use hue saturation value for the object detection
     */
