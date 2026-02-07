// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathfindToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.belt.BeltIO;
import frc.robot.subsystems.belt.BeltSimulationIO;
import frc.robot.subsystems.belt.BeltTalonFXIO;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbSimulationIO;
import frc.robot.subsystems.climb.ClimbTalonFXIO;
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
import frc.robot.subsystems.intake.IntakeTalonFXIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSimulationIO;
import frc.robot.subsystems.shooter.ShooterTalonFXIO;
import frc.robot.commands.communication.ControllerVibrateCommand;
import frc.robot.commands.communication.TellCommand;

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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // usign this for sys id so it doesn't conflict with anything
  @SuppressWarnings("unused")
  private final CommandXboxController sysID_contorller = new CommandXboxController(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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

        intake =
            new Intake(new IntakeIO() {});

        belt = new Belt(new BeltIO() {});

        shooter =
            new Shooter(
                new ShooterIO() {},
                () -> drive.getDistanceToGoal(),
                () -> drive.isWithinShooterAutomaticChargingZone());

        climb = new Climb(new ClimbIO() {});

        feeder = new Feeder(new FeederIO() {
          
        }, () -> drive.isPointingToGoal() && !drive.isSkidding(), () -> shooter.isOnTarget(), () -> true);

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
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
                () -> drive.getDistanceToGoal(),
                () -> drive.isWithinShooterAutomaticChargingZone());

        feeder =
            new Feeder(
                new FeederSimulationIO(),
                () -> drive.isPointingToGoal() && !drive.isSkidding(),
                () -> shooter.isOnTarget(),
                () -> true);

        climb = new Climb(new ClimbSimulationIO());

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

        break;
    }

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
    registerNamedCommandsAuto(); // register named commands for auto (pathplanner)
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .rightStick()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.fromDegrees(45)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    intake.setDefaultCommand(intake.setRollerVoltageAndPositionCommand(0, 0));
    belt.setDefaultCommand(belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER));
    feeder.setDefaultCommand(feeder.setPercentMotorRunCommand(0));
    shooter.setDefaultCommand(
        shooter.automaticallyChargeWhenNeededRunCommand(
            ShooterConstants.PERCENTAGE_OF_DISTANCE_WHEN_CHARGING, ShooterConstants.DEFAULT_SPEED));

    climb.setDefaultCommand(climb.setPositionsRunCommand(0, 0));

    Command aimTowardsTargetCommand =
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> -controller.getLeftX() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> drive.getHeadingToGoal());

    controller
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                aimTowardsTargetCommand,
                shooter.setAutomaticCommandRun(),
                feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER)));

    controller
        .rightTrigger()
        .whileTrue(
            intake.setRollerVoltageAndPositionCommand(
                IntakeConstants.INTAKE_POSITION, IntakeConstants.INTAKE_SPEED));

    controller.leftBumper().whileTrue(climb.setSpeedsRunCommand(1, 0.5));

    controller.a().onTrue(climb.setIsLockedCommand(() -> !climb.isLocked()));

    controller.b().whileTrue(new PathfindToPoseCommand(drive, () -> new Pose2d(), true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    String osName = System.getProperty("os.name").toLowerCase();
    if (osName.contains("win")) {
      // Windows
      return autoChooser.get();
    } else if (osName.contains("nix") || osName.contains("nux")) {
      int station = DriverStation.getLocation().orElse(1);
      switch (station) {
          // switches paths easily on linux since no smart dashboard
        case 1:
          //   return new PathPlannerAuto("auto left feeder station");
          return new PathPlannerAuto("get balls from middle (left)");
        case 2:
          return new PathPlannerAuto("auto middle feeder station2 twice, climb");
        case 3:
          return new PathPlannerAuto("auto right feeder station2 twice, climb");
        default:
          return new PathPlannerAuto("auto left feeder station");
      }
    } else {
      return autoChooser.get();
    }
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

  private void registerNamedCommandsAuto() {
    boolean isReal = true;
    // if (Constants.currentMode == Mode.SIM) isReal = false;

    //feed commands
    addNamedCommand("feedStart", feeder.setPercentMotorRunCommand(FeederConstants.FEED_POWER), isReal);
    addNamedCommand("feedWhenValid", feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER), isReal);
    addNamedCommand("feedStop", feeder.setPercentMotorRunCommand(0), isReal);

    addNamedCommand("beltStart", belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER), isReal);
    addNamedCommand("beltStop", belt.setPercentMotorOutputRunCommand(0), isReal);

    addNamedCommand("intakeStart", intake.setRollerVoltageAndPositionCommand(IntakeConstants.INTAKE_POSITION, IntakeConstants.INTAKE_SPEED), isReal);
    addNamedCommand("intakeStop", intake.setRollerVoltageAndPositionCommand(0, 0), isReal);
  }
 
  public void addNamedCommand(String commandName, Command command, boolean isReal) {

    if (isReal) {
      NamedCommands.registerCommand(
          commandName, command.andThen(new TellCommand("just ran " + commandName)));
      //   new EventTrigger(commandName).onTrue(command);
    } else {
      // registers the named commands to print something out instead of actually running anything
      NamedCommands.registerCommand(
          commandName,
          new TellCommand(commandName + " auto command")
              .andThen(
                  new ControllerVibrateCommand(1, controller).withDeadline(new WaitCommand(0.2)))
              .alongWith(command));

      //   new EventTrigger(commandName)
      //       .onTrue(
      //           new TellCommand(commandName + " auto event trigger command")
      //               .andThen(
      //                   new ControllerVibrateCommand(1, controller)
      //                       .withDeadline(new WaitCommand(0.2)))
      //               .andThen(new WaitCommand(0.3)));
    }
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
