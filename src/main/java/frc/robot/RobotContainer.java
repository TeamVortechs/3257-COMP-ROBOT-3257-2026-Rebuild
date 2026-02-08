// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.belt.BeltIO;
import frc.robot.subsystems.belt.BeltSimulationIO;
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
import frc.robot.subsystems.intake.IntakeTalonFXOnlyRollerIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSimulationIO;
import frc.robot.subsystems.shooter.ShooterTalonFXIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Intake intake;

  private final Belt belt;

  private final Feeder feeder;

  private final Shooter shooter;

  private final Climb climb;

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

  private static void flipAllPoses() {
    Constants.DriveConstants.SWICH_PASSING_GOALS = true;
    Constants.DriveConstants.PASSING_GOALS();
  }

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
            new Intake(
                new IntakeTalonFXOnlyRollerIO(
                    IntakeConstants.ROLLER_ID, IntakeConstants.POSITION_ID) {});

        belt = new Belt(new BeltIO() {});

        shooter =
            new Shooter(
                new ShooterTalonFXIO(ShooterConstants.MOTOR_ID),
                () -> drive.getDistanceToGoal(),
                () -> drive.isWithinShooterAutomaticChargingZone());

        climb = new Climb(new ClimbIO() {});

        feeder =
            new Feeder(
                new FeederTalonFXIO(FeederConstants.MOTOR_ID) {},
                () -> drive.isPointingToGoal() && !drive.isSkidding(),
                () -> shooter.isOnTarget(),
                () -> true);

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
                () -> drive.getDistanceToTarget(),
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

    SmartDashboard.putData("Flip Poses", Commands.runOnce(() -> flipAllPoses()));

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

    // intake
    intake.setDefaultCommand(intake.setRollerVoltageAndPositionCommand(0, 0));

    controller
        .leftTrigger()
        .whileTrue(intake.setRollerVoltageCommand(IntakeConstants.INTAKE_VOLTS));

    controller.rightTrigger().whileTrue(feeder.setPercentMotorRunCommand(0.4));

    controller.rightBumper().whileTrue(shooter.setManualSpeedRunCommand(70));
    controller.leftBumper().whileTrue(shooter.setAutomaticCommandRun());

    configureSysIdBindings(sysID_controller, shooter.BuildSysIdRoutine());

    // belt.setDefaultCommand(belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER));
    feeder.setDefaultCommand(feeder.setPercentMotorRunCommand(0));
    shooter.setDefaultCommand(shooter.setManualSpeedRunCommand(0));

    // climb.setDefaultCommand(climb.setPositionsRunCommand(0, 0));

    @SuppressWarnings("unused")
    Command aimTowardsTargetCommand =
        drive.joystickDriveAtTarget(
            drive,
            () -> -controller.getLeftY() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> -controller.getLeftX() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING);

    // controller
    //     .povUp()
    //     .whileTrue(
    //         Commands.parallel(
    //             aimTowardsTargetCommand,
    //             shooter.setAutomaticCommandRun(),
    //             feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER)));

    // controller.povRight().toggleOnTrue(drive.iteratePassingCommand(true));
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
