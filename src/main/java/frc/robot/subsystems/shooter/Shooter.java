package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // this shouldn't be here but it is for now because we're probably gonna move this
  public static final double TOLERANCE = ShooterConstants.TOLERANCE;

  private DoubleSupplier distanceSupplier;

  // just here for the logging
  @AutoLogOutput private double automaticSpeed = 0;
  @AutoLogOutput private double scaledAutomaticSpeed = 0;
  @AutoLogOutput private double automaticSpeedScalar = 1;

  @AutoLogOutput private boolean isManual = true;
  @AutoLogOutput private double manualSpeed = 0;

  // autologged like this so it can go on our dashboard
  @AutoLogOutput(key = "Shooter/ShooterTargetSpeed")
  private double targetSpeed = 0;

  @AutoLogOutput(key = "Shooter/ShooterSpeed")
  private double speed = 0;

  @AutoLogOutput(key = "Shooter/ShooterIsOnTarget")
  private boolean isOnTarget = false;

  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged inputs;

  // TO DO: add values to table
  private final InterpolatingDoubleTreeMap distToSpeedTable;

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
    this.speedToTableInit(10.0, 10.0); // dummy val
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("shooter", inputs);

    // calculate speed that automatically updates with distance
    automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());
    scaledAutomaticSpeed = automaticSpeed * automaticSpeedScalar;

    targetSpeed = getSpeedTarget();
    shooterIO.setSpeed(targetSpeed);

    speed = shooterIO.getSpeed();

    isOnTarget = isOnTarget();
  }

  // SUBSYSTEM METHODS

  /**
   * Sets the robot to use the distance supplier to determine shooting speed, the scalar determines
   * what scale of automatic speed to use
   */
  public void setAutomatic(double scalar) {
    isManual = false;
    this.automaticSpeedScalar = scalar;
  }

  /** Sets the robot to use the distance supplier to determine shooting speed, sclar of 1 */
  public void setAutomatic() {
    setAutomatic(1);
  }

  /**
   * @param speed the speed the flywheel will pid too
   */
  public void setManualSpeed(double speed) {
    this.isManual = true;
    this.manualSpeed = speed;
  }

  /**
   * @return the target speed that the flywheel is pid'ing to. Can be the manual or the automatic
   *     calculated speed
   */
  public double getSpeedTarget() {
    if (isManual) return manualSpeed;
    return scaledAutomaticSpeed;
  }

  /**
   * @return wether the speed is the target speed
   */
  public boolean isOnTarget() {
    return shooterIO.isOnTarget();
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
    return Commands.run(() -> this.setManualSpeed(speed), this);
  }

  /**
   * sets the target speed command then ends when it reaches that speed
   *
   * @param speed the speed it gets set to
   * @return the finished command
   */
  public Command setManualSpeedCommandConsistentEnd(double speed) {
    return new InstantCommand(() -> this.setManualSpeed(speed), this)
        .andThen(new WaitUntilCommand(() -> this.isOnTarget()));
  }

  /**
   * returns a command that revs up to shoot at the distance and ends immediately
   *
   * @return
   */
  public Command setAutomaticCommand() {
    return new InstantCommand(() -> this.setAutomatic(), this);
  }

  public Command setAutomaticCommandRun() {
    return Commands.run(() -> this.setAutomatic(), this);
  }

  /**
   * returns a command that revs up to shoot at the distance then ends when it reaches that point
   *
   * @return
   */
  public Command setAutomaticCommandConsistentEnd() {
    return new InstantCommand(() -> this.setAutomatic(), this)
        .andThen(new WaitUntilCommand(() -> this.isOnTarget()));
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
    return distance;

    // commented otu for now becaues this is flat since we don't have values, I want something we
    // can debug from
    // return this.distToSpeedTable.get(distance);
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

  // this shoudl be in a helper method or somewhere in robot container
  /**
   * Binds the sys id routine to the controller so we can use it
   *
   * <p>pov up: dynamic forward pov down: dynamic backwards pov right: quasistatic forward pov left:
   * quasistatic reverse
   *
   * @param controller the controller this binds to(recommended to use a high id controller to
   *     prevent mishaps, id 2-3)
   * @param sysIdRoutine the routine that this controller will activate
   */
  public void configureSysIdBindings(CommandXboxController controller, SysIdRoutine sysIdRoutine) {
    controller.povUp().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    controller.povDown().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    controller.povRight().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    controller.povLeft().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));
  }
}

/*
figure out a robot lead system to allow shooting on the move
figure out a leading compensation system to shooting on the move


Shooter corridor where shooting is allowed?

limit angular velocity, acceleration, and especially jerk when shooting
this stability should be checked for over a window


Feed allowed if all are true:
tag visible (or pose confidence high)


heading error < θThresh


RPM error < rpmThresh


abs(omega) < omegaThresh


accelMag < aThresh (or stable window)

if (rpmError < tolerance && rpmDerivative > -maxDropRate) {
    feed();
}

maybe filter distance
And don’t run the feeder until feedAllowed has been continuously true for ~100–200ms.



only have the shooter strand winding up when close scoring location and battery voltage is high enough

add this code to detect when we are moving closer/farther away from note
rangeRate = dot(v, r̂)
rpmTarget += kRangeRate * rangeRate

slip detection

 */

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
