// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.photon0Name;
import static frc.robot.subsystems.vision.VisionConstants.photon1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToPhoton0;
import static frc.robot.subsystems.vision.VisionConstants.robotToPhoton1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.belt.BeltIO;
import frc.robot.subsystems.belt.BeltSimulationIO;
import frc.robot.subsystems.belt.BeltTalonFXIO;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbSimulationIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederSimulationIO;
import frc.robot.subsystems.feeder.FeederTalonFXIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeSimulationIO;
import frc.robot.subsystems.intake.IntakeTalonFXCANCoderIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSimulationIO;
import frc.robot.subsystems.shooter.ShooterTalonFXIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.MatchTimeline;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Intake intake;

  private final Belt belt;

  private final Feeder feeder;

  private final Shooter shooter;

  private final Climb climb;

  private final Vision vision;

  // private final Climb climb;

  // Controller

  @SuppressWarnings("unused")
  private final CommandXboxController controller = new CommandXboxController(0);

  @SuppressWarnings("unused")
  private final CommandXboxController operatorController = new CommandXboxController(1);

  @SuppressWarnings("unused")
  private final CommandXboxController testController = new CommandXboxController(2);

  @SuppressWarnings("unused")
  private final CommandXboxController sysID_controller = new CommandXboxController(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final MatchTimeline matchTimeline = new MatchTimeline(operatorController, controller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURR_MODE) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        intake =
            new Intake(
                new IntakeTalonFXCANCoderIO(
                    IntakeConstants.INTAKE_ROLLER_MOTOR_ID,
                    IntakeConstants.INTAKE_POSITION_MOTOR_ID,
                    IntakeConstants.INTAKE_CANCODER_ID));
        // intake = new Intake(new IntakeIO() {});

        shooter =
            new Shooter(
                new ShooterTalonFXIO(ShooterConstants.MOTOR_ID),
                () -> drive.getDistanceToGoal(),
                () -> drive.isWithinShooterAutomaticChargingZone());
        // shooter =
        //     new Shooter(
        //         new ShooterIO() {},
        //         () -> drive.getDistanceToGoal(),
        //         () -> drive.isWithinShooterAutomaticChargingZone());

        feeder =
            new Feeder(
                new FeederTalonFXIO(FeederConstants.MOTOR_ID),
                () -> drive.isPointingToGoal(),
                () -> shooter.isOnTarget(),
                () -> true);
        // feeder =
        //     new Feeder(
        //         new FeederIO() {},
        //         () -> drive.isPointingToGoal(),
        //         () -> shooter.isOnTarget(),
        //         () -> true);

        belt = new Belt(new BeltTalonFXIO(BeltConstants.ID));
        // belt = new Belt(new BeltIO() {});

        climb = new Climb(new ClimbIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(photon0Name, robotToPhoton0),
                new VisionIOPhotonVision(photon1Name, robotToPhoton1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new Intake(new IntakeSimulationIO());

        belt = new Belt(new BeltSimulationIO());

        shooter =
            new Shooter(
                new ShooterSimulationIO(),
                () -> drive.getDistanceToTarget(),
                () -> drive.isWithinShooterAutomaticChargingZone());

        feeder =
            new Feeder(
                new FeederSimulationIO(),
                () -> drive.isPointingToGoal(),
                () -> shooter.isOnTarget(),
                () -> true);

        climb = new Climb(new ClimbSimulationIO());

        // climb = new Climb(new ClimbSimulationIO());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.photon0Name, VisionConstants.robotToPhoton0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.photon1Name, VisionConstants.robotToPhoton1, drive::getPose));

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new Intake(new IntakeIO() {});

        belt = new Belt(new BeltIO() {});

        feeder = new Feeder(new FeederIO() {}, () -> false, () -> false, () -> false);

        shooter =
            new Shooter(
                new ShooterIO() {},
                () -> drive.getDistanceToGoal(),
                () -> drive.isWithinShooterAutomaticChargingZone());

        climb = new Climb(new ClimbIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        break;
    }

    registerNamedCommandsAuto(); // register named commands for auto (pathplanner)

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // matchTimeline.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // default commands
    shooter.setDefaultCommand(shooter.automaticallyChargeWhenNeededRunCommand(0, 0));
    intake.setDefaultCommand(intake.setRollerVoltageCommand(0));
    feeder.setDefaultCommand(feeder.setPercentMotorRunCommand(0));
    climb.setDefaultCommand(climb.setVoltageRun(0));
    belt.setDefaultCommand(belt.setPercentMotorOutputCommand(BeltConstants.DEFAULT_POWER));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX() * 0.7));

    // CONTROLLER:

    // Lock to 0° when A button is held
    controller
        .rightStick()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> drive.getRotationOverBumper()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .povUp()
        .whileTrue(intake.setPositionCommand(IntakeConstants.INTAKE_HALFWAY_UP_POSITION));

    controller.povRight().whileTrue(intake.setPositionCommand(0));
    controller.povLeft().whileTrue(drive.iteratePassingCommand(true));

    @SuppressWarnings("unused")
    Command aimTowardsTargetCommand =
        drive.joystickDriveAtTarget(
            drive,
            () -> -controller.getLeftY() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> -controller.getLeftX() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING);

    @SuppressWarnings("unused")
    Command aimTowardsTargetCommand2 =
        drive.joystickDriveAtTarget(
            drive,
            () -> -controller.getLeftY() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> -controller.getLeftX() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING);

    // shoot commands
    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                    aimTowardsTargetCommand,
                    shooter.setAutomaticCommandRun(),
                    belt.setPercentMotorOutputRunCommand(
                        BeltConstants.FEED_POWER, () -> feeder.getTargetSpeed() > 0),
                    feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller
        .b()
        .whileTrue(
            Commands.parallel(aimTowardsTargetCommand2, shooter.setAutomaticCommandRun())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller.povRight().whileTrue(feeder.setPercentMotorRunCommand(1));

    controller
        .rightBumper()
        .onTrue(shooter.setAutomaticallyChargeFully(() -> true))
        .onFalse(shooter.setAutomaticallyChargeFully(() -> false));

    // eject balls
    controller
        .a()
        .whileTrue(
            Commands.parallel(
                intake.setRollerVoltageAndPositionCommand(
                    IntakeConstants.INTAKE_DOWN_POSITION, IntakeConstants.EJECT_VOLTS)));
    // intake command
    controller
        .leftTrigger()
        .whileTrue(
            intake
                .setPositionAndRollersCommandConsistentEnd(
                    IntakeConstants.INTAKE_DOWN_POSITION, IntakeConstants.ROLLER_GOING_DOWN_VOLTS)
                // .andThen(new PrintCommand("it ended"))
                .andThen(intake.setRollerVoltageCommand(IntakeConstants.INTAKE_VOLTS)));

    // intake up position
    controller
        .leftBumper()
        .onTrue(
            intake.setPositionAndRollersCommandConsistentEnd(
                IntakeConstants.INTAKE_HALFWAY_UP_POSITION, IntakeConstants.ROLLER_GOING_UP_VOLTS));

    controller.start().whileTrue(new InstantCommand(() -> drive.setPose(new Pose2d())));

    controller.x().whileTrue(drive.stayAtPoseCommand());

    // operator controller
    // climb
    operatorController
        .leftStick()
        .whileTrue(
            climb.setVoltageRun(
                () -> operatorController.getLeftY() * ClimbConstants.CLIMB_UP_VOLTS));

    operatorController.leftBumper().onTrue(climb.setLockedInstant(true));
    operatorController.leftTrigger().onTrue(climb.setLockedInstant(false));

    operatorController.rightStick().whileTrue(drive.iteratePassingCommand(true));

    operatorController
        .rightBumper()
        .whileTrue(
            feeder
                .setPercentMotorRunCommand(FeederConstants.FEED_POWER)
                .alongWith(belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER)));
    operatorController.rightTrigger().whileTrue(shooter.setManualSpeedRunCommand(78));

    // operatorController
    //     .b()
    //     .onTrue(
    //
    // intake.resetEncoderInstant(IntakeConstants.INTAKE_UP_POSITION).ignoringDisable(true));

    // operatorController
    //     .b()
    //     .onTrue(
    //         intake.setPositionAndRollersCommandConsistentEnd(
    //             IntakeConstants.INTAKE_UP_POSITION, IntakeConstants.ROLLER_GOING_UP_VOLTS));
    operatorController
        .x()
        .whileTrue(
            feeder
                .setPercentMotorRunCommand(-0.3)
                .alongWith(shooter.setManualSpeedRunCommand(-10)));

    operatorController
        .y()
        .whileTrue(
            shooter.setAutomaticallyChargeFully(() -> !shooter.isAutomaticallyChargeFully()));

    operatorController.povUp().whileTrue(belt.setPercentMotorOutputRunCommand(0.5));
    operatorController.povDown().whileTrue(belt.setPercentMotorOutputRunCommand(-0.2));

    operatorController.povRight().onTrue(drive.setPassingIndexCommmand(1));
    operatorController.povLeft().onTrue(drive.setPassingIndexCommmand(0));

    testController.a().whileTrue(feeder.setPercentMotorRunCommand(FeederConstants.FEED_POWER));
    testController.b().whileTrue(belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER));

    testController
        .x()
        .whileTrue(
            shooter
                .setManualSpeedRunCommand(() -> ShooterConstants.SHOOTER_TEST_SPEED.get())
                .alongWith(feeder.feedWhenValidRunCommand(1))
                .alongWith(
                    belt.setPercentMotorOutputRunCommand(
                        BeltConstants.FEED_POWER, () -> feeder.getTargetSpeed() > 0)));

    // sysid bindings:[]\

    configureSysIdBindings(sysID_controller, shooter.BuildSysIdRoutine());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Intake getIntake() {
    return intake;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Drive getDrive() {
    return drive;
  }

  public MatchTimeline getMatchTimeline() {
    return matchTimeline;
  }

  private void registerNamedCommandsAuto() {

    NamedCommands.registerCommand("shooterAutomatic", shooter.setAutomaticCommandRun());
    NamedCommands.registerCommand("shooterStop", shooter.setManualSpeedCommand(0));

    NamedCommands.registerCommand(
        "feedWhenValid", feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER));

    NamedCommands.registerCommand(
        "feedWhenValidAndStop",
        Commands.race(
            // first command(feedwhen valid and stop)
            new WaitUntilCommand(() -> shooter.isOnTarget())
                .andThen(
                    feeder
                        .feedWhenValidRunCommand(FeederConstants.FEED_POWER)
                        .withDeadline(new WaitCommand(3)))
                .andThen(feeder.setPercentMotorCommand(0)),
            // second command(point towards target)
            drive.joystickDriveAtTarget(drive, () -> 0, () -> 0),
            belt.setPercentMotorOutputRunCommand(
                BeltConstants.FEED_POWER, () -> feeder.getTargetSpeed() > 0),
            shooter.setAutomaticCommandRun())
            .andThen(shooter.setManualSpeedCommand(0)));

    /*
    version with moving intake:
         NamedCommands.registerCommand(
        "feedWhenValidAndStop",
        new WaitUntilCommand(() -> shooter.isOnTarget())
            .andThen(
                feeder
                    .feedWhenValidRunCommand(FeederConstants.FEED_POWER)
                    .withDeadline(intake.intakeRetractWhileShooting(new InstantCommand(), 4)))
            .andThen(feeder.setPercentMotorCommand(0)));
     */

    NamedCommands.registerCommand("feedStop", feeder.setPercentMotorCommand(0));

    NamedCommands.registerCommand(
        "intakeStart", intake.setRollerVoltageCommand(IntakeConstants.INTAKE_VOLTS));

    NamedCommands.registerCommand(
        "intakeStartAndDown",
        intake.setRollerVoltageAndPositionCommand(
            IntakeConstants.INTAKE_DOWN_POSITION, IntakeConstants.INTAKE_VOLTS));

    NamedCommands.registerCommand(
        "intakeStop", new InstantCommand(() -> intake.setRollersVoltage(0)));

    NamedCommands.registerCommand(
        "intakeDown",
        intake
            .setPositionCommand(IntakeConstants.INTAKE_DOWN_POSITION)
            .withDeadline(new WaitUntilCommand(() -> intake.isOnTarget())));

    NamedCommands.registerCommand(
        "intakeUp",
        intake
            .setPositionCommand(IntakeConstants.INTAKE_HALFWAY_UP_POSITION)
            .withDeadline(new WaitUntilCommand(() -> intake.isOnTarget())));

    NamedCommands.registerCommand("shooterStop", shooter.setManualSpeedCommand(0));

    NamedCommands.registerCommand(
        "waitForShooterOnTarget", new WaitUntilCommand(() -> shooter.isOnTarget()));

    NamedCommands.registerCommand("driveOverrideRotation", drive.overrideRotationCommand());
    // new EventTrigger("driveOverrideRotation").onTrue(drive.overrideRotationCommand());

    NamedCommands.registerCommand("driveResetOverrides", drive.removeRotationOverrideCommand());

    NamedCommands.registerCommand("climbUp", climb.setVoltageRun(ClimbConstants.CLIMB_UP_VOLTS));
    NamedCommands.registerCommand("climbStop", climb.setVoltageInstant(0));
    NamedCommands.registerCommand(
        "climbDownWhenNeeded",
        new WaitUntilCommand(() -> matchTimeline.getTimeSinceStart() > 18)
            .andThen(climb.setVoltageRun(ClimbConstants.CLIMB_DOWN_VOLTS)));

    new EventTrigger("intakeStartEvent")
        .onTrue(
            new InstantCommand(
                () -> intake.setRollersVoltage(Constants.IntakeConstants.INTAKE_VOLTS)));
    new EventTrigger("intakeStopEvent")
        .onTrue(new InstantCommand(() -> intake.setRollersVoltage(0)));
    new EventTrigger("intakeDownEvent")
        .onTrue(
            new InstantCommand(() -> intake.setPosition(IntakeConstants.INTAKE_DOWN_POSITION))
                .alongWith(
                    new InstantCommand(
                        () -> intake.setRollersVoltage(IntakeConstants.ROLLER_GOING_DOWN_VOLTS))));

    new EventTrigger("shootOnMove")
        //makes the drive rotate correctly
        .onTrue(drive.overrideRotationCommand())
        .onFalse(drive.removeRotationOverrideCommand())

        //starts shooter + feeder + belts
        .whileTrue(Commands.parallel(new RunCommand(() -> {shooter.setAutomaticSpeed(1);}),
        feeder.feedWhenValidRunCommandAutoEvent(FeederConstants.FEED_POWER),
        belt.setPercentMotorOutputRunCommandAutoEvent(1, () -> feeder.getTargetSpeed() > 0)));

    new EventTrigger("revShooterEvent")
        .whileTrue(
            new RunCommand(
                () -> {
                  shooter.setAutomaticSpeed(1);
                }));

    new EventTrigger("intakeUpEvent")
        .onTrue(
            new InstantCommand(() -> intake.setPosition(0))
                .andThen(intake.setRollerVoltageCommand(IntakeConstants.ROLLER_GOING_UP_VOLTS)));

    new EventTrigger("climbUpEvent")
        .onTrue(new InstantCommand(() -> climb.setVoltage(ClimbConstants.CLIMB_UP_VOLTS)));
    new EventTrigger("climbStopEvent").onTrue(new InstantCommand(() -> climb.setVoltage(0)));

    new EventTrigger("reverseFeederEvent")
        .onTrue(
            new InstantCommand(
                () -> feeder.setPercentMotorOutput(FeederConstants.EJECT_POWER_AUTO)));
    new EventTrigger("stopFeederEvent")
        .onTrue(new InstantCommand(() -> feeder.setPercentMotorOutput(0)));
  }

  // this shoudl be in a helper method or somewhere in robot container
  /**
   * y: dynamic forward a: dynamic backwards b: quasistatic forward x: quasistatic reverse
   *
   * @param controller the controller this binds to(recommended to use a high id controller to
   *     prevent mishaps, id 2-3)
   * @param sysIdRoutine the routine that this controller will activate
   */
  public void configureSysIdBindings(CommandXboxController controller, SysIdRoutine sysIdRoutine) {
    controller.y().whileTrue(sysIdRoutine.dynamic(Direction.kForward));
    controller.a().whileTrue(sysIdRoutine.dynamic(Direction.kReverse));
    controller.b().whileTrue(sysIdRoutine.quasistatic(Direction.kForward));
    controller.x().whileTrue(sysIdRoutine.quasistatic(Direction.kReverse));
  }
}
