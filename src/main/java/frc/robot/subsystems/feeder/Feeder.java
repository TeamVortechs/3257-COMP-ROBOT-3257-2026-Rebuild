package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FeederConstants;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private FeederIO feederIO;
  private FeederIOInputsAutoLogged inputs;

  /**
   * @param feederIO the hardware interface
   */
  public Feeder(FeederIO feederIO) {
    this.feederIO = feederIO;
    this.inputs = new FeederIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(inputs);
    Logger.processInputs("feeder", inputs);

    // calculate speed that automatically updates with distance
    // automaticSpeed = getSpeedFromDistance(distanceSupplier.getAsDouble());
  }

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

  // COMMANDS
  /**
   * sets the manual speed of the flywheel then ends immediately
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorCommand(double speed) {
    return new InstantCommand(() -> this.setPercentMotorOutput(speed));
  }

  /**
   * sets the manual speed of the flywheel, runs multiple times
   *
   * @param speed the speed of the flywheel
   * @return the finished command
   */
  public Command setPercentMotorRunCommand(double speed) {
    return Commands.run(() -> this.setPercentMotorOutput(speed), this);
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
