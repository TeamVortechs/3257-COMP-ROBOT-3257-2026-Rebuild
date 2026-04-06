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
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PathfindToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.belt.Belt;
import frc.robot.subsystems.belt.BeltIO;
import frc.robot.subsystems.belt.BeltSimulationIO;
import frc.robot.subsystems.belt.BeltTalonFXIO;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.DrivetrainIO;
import frc.robot.subsystems.drive.DrivetrainSimulationIO;
import frc.robot.subsystems.drive.DrivetrainTalonFXIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederSimulationIO;
import frc.robot.subsystems.feeder.FeederTalonFXIO;
import frc.robot.subsystems.feeder.FeederValidityContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeSimulationIO;
import frc.robot.subsystems.intake.IntakeTalonFXCANCoderIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSimulationIO;
import frc.robot.subsystems.shooter.ShooterTalonFXIO;
import frc.robot.subsystems.vision.PowerModuleIO;
import frc.robot.subsystems.vision.PowerModuleIORev;
import frc.robot.subsystems.vision.PowerModuleIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.VortechsController;
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
  private final Drivetrain drive;

  private final Intake intake;

  private final Belt belt;

  private final Feeder feeder;

  private final Shooter shooter;

  private final Vision vision;

  // private final Climb climb;

  // Controller

  @SuppressWarnings("unused")
  private final VortechsController controller = new VortechsController(0);

  @SuppressWarnings("unused")
  private final VortechsController operatorController = new VortechsController(1);

  @SuppressWarnings("unused")
  private final VortechsController testController = new VortechsController(2);

  @SuppressWarnings("unused")
  private final VortechsController sysID_controller = new VortechsController(3);

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
            new Drivetrain(
                new DrivetrainTalonFXIO(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));

        intake =
            new Intake(
                new IntakeTalonFXCANCoderIO(
                    IntakeConstants.INTAKE_ROLLER_MOTOR_ID,
                    IntakeConstants.INTAKE_ROLLER_2_MOTOR_ID,
                    IntakeConstants.INTAKE_POSITION_MOTOR_ID,
                    IntakeConstants.INTAKE_CANCODER_ID));
        // intake = new Intake(new IntakeIO() {});
        // intake =
        //     new Intake(
        //         new IntakeTalonFXOnlyRollerIO(
        //             IntakeConstants.INTAKE_ROLLER_MOTOR_ID,
        //             IntakeConstants.INTAKE_POSITION_MOTOR_ID));

        shooter =
            new Shooter(
                new ShooterTalonFXIO(
                    ShooterConstants.MOTOR_ID,
                    ShooterConstants.FOLLOWER_MOTOR_ID,
                    ShooterConstants.FOLLOWER_2_MOTOR_ID),
                () -> drive.getDistanceToTarget());
        // shooter = new Shooter(new ShooterIO() {}, () -> drive.getDistanceToTarget());

        feeder =
            new Feeder(
                new FeederTalonFXIO(FeederConstants.MOTOR_ID),
                new FeederValidityContainer(
                    () -> drive.isOriented(),
                    () -> shooter.isOnTarget(),
                    () -> true,
                    operatorController.leftTrigger(),
                    () -> drive.isInScoringZone())); // feeder =
        //     new Feeder(
        //         new FeederIO() {},
        //         () -> drive.isPointingToGoal(),
        //         () -> shooter.isOnTarget(),
        //         () -> true);

        belt = new Belt(new BeltTalonFXIO(BeltConstants.ID));
        // belt = new Belt(new BeltIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new PowerModuleIORev(),
                new VisionIOPhotonVision(photon0Name, robotToPhoton0),
                new VisionIOPhotonVision(photon1Name, robotToPhoton1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive =
            new Drivetrain(
                new DrivetrainSimulationIO(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));

        intake = new Intake(new IntakeSimulationIO());

        belt = new Belt(new BeltSimulationIO());

        shooter = new Shooter(new ShooterSimulationIO(), () -> drive.getDistanceToTarget());

        feeder =
            new Feeder(
                new FeederSimulationIO(),
                new FeederValidityContainer(
                    () -> drive.isOriented(),
                    () -> shooter.isOnTarget(),
                    () -> true,
                    operatorController.leftTrigger(),
                    () -> drive.isInScoringZone()));
        // climb = new Climb(new ClimbSimulationIO());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new PowerModuleIOSim(),
                new VisionIOPhotonVisionSim(
                    VisionConstants.photon0Name, VisionConstants.robotToPhoton0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.photon1Name, VisionConstants.robotToPhoton1, drive::getPose));

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        break;

      default:
        drive = new Drivetrain(new DrivetrainIO() {});

        // Replayed robot, disable IO implementations

        intake = new Intake(new IntakeIO() {});

        belt = new Belt(new BeltIO() {});

        feeder =
            new Feeder(
                new FeederIO() {},
                new FeederValidityContainer(
                    () -> false, () -> false, () -> false, () -> false, () -> false));

        shooter = new Shooter(new ShooterIO() {}, () -> drive.getDistanceToTarget());

        vision = new Vision(drive::addVisionMeasurement, new PowerModuleIO() {}, new VisionIO() {});

        break;
    }

    registerNamedCommandsAuto(); // register named commands for auto (pathplanner)

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    shooter.setDefaultCommand(shooter.defaultCommand());
    intake.setDefaultCommand(intake.setRollerVoltageCommand(0));
    feeder.setDefaultCommand(feeder.setPercentMotorRunCommand(0));
    belt.setDefaultCommand(belt.setPercentMotorOutputCommand(BeltConstants.DEFAULT_POWER));

    // Default command, normal field-relative drive

    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // CONTROLLER:

    // Lock to 0° when A button is held
    controller
        .rightStick()
        .whileTrue(
            drive.joystickDriveAtTarget(
                () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller
        .rightStick()
        .whileTrue(
            drive.joystickDriveRotation(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d(-controller.getLeftY(), -controller.getLeftX()).times(-1)));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .povUp()
        .whileTrue(intake.setPositionCommand(IntakeConstants.INTAKE_HALFWAY_UP_POSITION));

    controller.povRight().whileTrue(intake.setPositionCommand(0));

    @SuppressWarnings("unused")
    Command aimTowardsTargetCommand =
        drive.joystickDriveAtTarget(
            // drive,
            () -> -controller.getLeftY() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
            () -> -controller.getLeftX() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING);

    @SuppressWarnings("unused")
    Command aimTowardsTargetCommand2 =
        drive.joystickDriveAtTarget(
            // drive,
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
                    feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER),
                    intake.intakeRetractWhileShooting(() -> feeder.getTargetSpeed() > 0))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    controller
        .y()
        .whileTrue(
            drive
                .joystickDriveRotation(
                    () -> -controller.getLeftY() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
                    () -> -controller.getLeftX() * DriveConstants.K_JOYSTICK_WHEN_SHOOTING,
                    () ->
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? Rotation2d.k180deg
                            : Rotation2d.kZero)
                .alongWith(
                    shooter
                        .setManualSpeedCommand(83)
                        .alongWith(
                            new WaitUntilCommand(() -> shooter.isOnTarget())
                                .andThen(
                                    feeder
                                        .setPercentMotorRunCommand(FeederConstants.FEED_POWER)
                                        .alongWith(
                                            intake.intakeRetractWhileShooting(
                                                () -> shooter.isOnTarget())))))
                .alongWith(
                    belt.setPercentMotorOutputRunCommand(
                        BeltConstants.FEED_POWER, () -> feeder.getTargetSpeed() > 0)));

    // controller
    //     .x()
    //     .whileTrue(
    //         shootAfterPathingCommand(
    //             new PathfindToPoseCommand(drive, DriveConstants.CLIMB_SHOOT_POSE_LEFT, true)));

    controller
        .x()
        .whileTrue(
            new ConditionalCommand(
                shootAfterPathingCommand(
                    new PathfindToPoseCommand(drive, DriveConstants.CLIMB_SHOOT_POSE_RIGHT, true)),
                shootAfterPathingCommand(
                    new PathfindToPoseCommand(drive, DriveConstants.CLIMB_SHOOT_POSE_LEFT, true)),
                () -> drive.isRightSideZone()));

    controller
        .b()
        .whileTrue(
            new ConditionalCommand(
                shootAfterPathingCommand(
                    new PathfindToPoseCommand(drive, DriveConstants.BUMPER_SHOOT_POSE_RIGHT, true)),
                shootAfterPathingCommand(
                    new PathfindToPoseCommand(drive, DriveConstants.BUMPER_SHOOT_POSE_LEFT, true)),
                () -> drive.isRightSideZone()));

    // controller
    //     .b()
    //     .whileTrue(
    //         shootAfterPathingCommand(
    //             new PathfindToPoseCommand(drive, DriveConstants.CLIMB_SHOOT_POSE_RIGHT, true)));

    PathPlannerPath leftSideProtectedShoot = getPath("Left Side Tele mid into score");
    PathPlannerPath rightSideProtectedShoot = getPath("Right Side Tele mid into score");

    // controller
    //     .y()
    //     .whileTrue(
    //         shootAfterPathingCommand(
    //             new ConditionalCommand(
    //                 AutoBuilder.pathfindThenFollowPath(
    //                     rightSideProtectedShoot, rightSideProtectedShoot.getGlobalConstraints()),
    //                 AutoBuilder.pathfindThenFollowPath(
    //                     leftSideProtectedShoot, leftSideProtectedShoot.getGlobalConstraints()),
    //                 () -> drive.isRightSideZone())));

    PathPlannerPath leftSideUnderClimb = getPath("Teleop Climber left to right");
    PathPlannerPath rightSideUnderClimb = getPath("Teleop Climber right to left");

    controller
        .rightBumper()
        .whileTrue(
            new ConditionalCommand(
                AutoBuilder.pathfindThenFollowPath(
                    rightSideUnderClimb, rightSideUnderClimb.getGlobalConstraints()),
                AutoBuilder.pathfindThenFollowPath(
                    leftSideUnderClimb, leftSideUnderClimb.getGlobalConstraints()),
                () -> drive.isRightSideZone()));

    // eject balls
    controller
        .a()
        .whileTrue(
            Commands.parallel(
                intake
                    .setRollerVoltageAndPositionCommand(
                        IntakeConstants.INTAKE_DOWN_POSITION, IntakeConstants.EJECT_VOLTS)
                    .alongWith(belt.setPercentMotorOutputCommand(BeltConstants.EJECT_POWER))));
    // intake command
    controller
        .leftTrigger()
        .whileTrue(
            intake
                .setPositionCommandConsistentEnd(IntakeConstants.INTAKE_DOWN_POSITION)
                .andThen(intake.setRollerVoltageCommand(IntakeConstants.INTAKE_VOLTS)));

    // intake up position
    controller
        .leftBumper()
        .onTrue(intake.setPositionCommand(IntakeConstants.INTAKE_HALFWAY_UP_POSITION));

    controller.start().whileTrue(new InstantCommand(() -> drive.resetPose(new Pose2d())));

    // operator controller

    // TEMP BIND: make the intake go up real slow
    operatorController
        .start()
        .whileTrue(
            intake.setPositionWithVelocityCommand(
                IntakeConstants.INTAKE_HALFWAY_UP_POSITION,
                IntakeConstants.MOTION_MAGIC_SLOWED_VELOCITY))
        .onFalse(new InstantCommand(() -> intake.setPositionVoltage(0), intake));

    // climb
    // operatorController
    //     .leftStick()
    //     .whileTrue(
    //         climb.setVoltageRun(
    //             () -> operatorController.getLeftY() * ClimbConstants.CLIMB_UP_VOLTS));

    // operatorController.leftBumper().onTrue(climb.setLockedInstant(true));
    // operatorController.leftTrigger().onTrue(climb.setLockedInstant(false));

    // these climb commands are going unused. i'll repurpose them for unsticking the intake
    // while holding down left stick,
    // left stick forward --> intake down
    // left stick back --> intake up
    operatorController
        .leftStick()
        .whileTrue(
            new RunCommand(
                () ->
                    intake.setPositionVoltage(
                        // square the up/down input for finer control
                        // after squaring, keep same sign as the input so up/down works
                        () -> // use the negative so forward = down and back = up
                        Math.signum(operatorController.getLeftY())
                                * Math.pow(operatorController.getLeftY(), 2)
                                *
                                // i would REALLY not want the intake to run at more than 2V but
                                // ngl if we're using this we might have to
                                IntakeConstants.MAX_MANUAL_VOLTS),
                intake))
        .onFalse(new InstantCommand(() -> intake.setPositionVoltage(0)));

    operatorController.rightTrigger().whileTrue(shooter.setManualSpeedRunCommand(78));

    operatorController.b().onTrue(vision.setPDHCommand(false)).onFalse(vision.setPDHCommand(true));

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

    operatorController.a().whileTrue(intake.setRollerVoltageCommand(12));

    operatorController
        .povUp()
        .whileTrue(intake.setPositionCommand(IntakeConstants.DEEP_INTAKE_UP_POSITION));
    operatorController
        .povDown()
        .whileTrue(intake.setPositionCommand(IntakeConstants.INTAKE_HALFWAY_LOWER_POSITION));

    // sysid bindings:
    // configureSysIdBindings(sysID_controller, shooter.BuildSysIdRoutine());

    sysID_controller
        .povUp()
        .whileTrue(
            Commands.parallel(
                    shooter.setManualSpeedRunCommand(
                        () -> ShooterConstants.SHOOTER_TEST_SPEED.get()),
                    belt.setPercentMotorOutputRunCommand(
                        BeltConstants.FEED_POWER, () -> feeder.getTargetSpeed() > 0),
                    feeder.feedWhenShooterIsRevvedCommand(FeederConstants.FEED_POWER),
                    intake.intakeRetractWhileShooting(() -> feeder.getTargetSpeed() > 0))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    sysID_controller.a().whileTrue(drive.sysIdDynamic(Direction.kReverse));
    sysID_controller.y().whileTrue(drive.sysIdDynamic(Direction.kForward));
    sysID_controller.x().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    sysID_controller.b().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    // additional testing bindings
    testController.a().whileTrue(feeder.setPercentMotorRunCommand(FeederConstants.FEED_POWER));
    testController.b().whileTrue(belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER));

    // testController
    //     .y()
    //     .whileTrue(
    //         shooter
    //             .setManualSpeedRunCommand(() -> ShooterConstants.SHOOTER_TEST_SPEED.get())
    //             .alongWith(
    //                 new WaitUntilCommand(() -> shooter.isOnTarget())
    //                     .andThen(
    //                         feeder
    //                             .setPercentMotorRunCommand(FeederConstants.FEED_POWER)
    //                             .alongWith(
    //                                 intake.intakeRetractWhileShooting(
    //                                     () -> feeder.getTargetSpeed() > 0)))));

    // 2.645

    // TODO: see if this will require adding InterruptBehavior of canceling any other intake
    // commands while the intake is running so that it doesn't scrindongulode our intake mid-way
    // through its travel
    // 3-19-26: mechanical said no. i'm keeping the command and its test here
    // in the case mechanical says yes in the future.
    // Command oscillateIntake =
    //     new SequentialCommandGroup(
    //             intake.setPositionWithVelocityAndRollersCommandConsistentEnd(
    //                 IntakeConstants.INTAKE_HALFWAY_UP_POSITION,
    //                 IntakeConstants.OSCILLATION_VELOCITY,
    //                 IntakeConstants.ROLLER_GOING_UP_VOLTS),
    //             new WaitCommand(IntakeConstants.WAIT_TIME_BETWEEN_INTAKE_OSCILLATION),
    //             intake.setPositionWithVelocityAndRollersCommandConsistentEnd(
    //                 IntakeConstants.INTAKE_DOWN_POSITION,
    //                 IntakeConstants.OSCILLATION_VELOCITY,
    //                 IntakeConstants.ROLLER_GOING_DOWN_VOLTS),
    //             new WaitCommand(IntakeConstants.WAIT_TIME_BETWEEN_INTAKE_OSCILLATION))
    //         .repeatedly();
    // testController
    //     .rightBumper()
    //     .whileTrue(
    //         oscillateIntake.alongWith( // not sure if running belt is desired
    //             belt.setPercentMotorOutputRunCommand(BeltConstants.FEED_POWER)));
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

  public Drivetrain getDrive() {
    return drive;
  }

  private void registerNamedCommandsAuto() {

    NamedCommands.registerCommand("shooterAutomatic", shooter.setAutomaticCommandRun());
    NamedCommands.registerCommand("shooterStop", shooter.setManualSpeedCommand(0));

    // NamedCommands.registerCommand("intakeDownFirstTime",
    // intake.setPositionVoltageRunCommand(6).withDeadline(new WaitUntilCommand(() ->
    // intake.getPosition() > IntakeConstants.INTAKE_HALFWAY_UP_POSITION)));

    NamedCommands.registerCommand(
        "feedWhenValid", feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER));

    NamedCommands.registerCommand(
        "feedWhenValidAndStop",
        Commands.race(
                // first part of the command, wait's until it is on target
                new WaitUntilCommand(() -> shooter.isOnTarget())

                    // feed for 3 secs
                    .andThen(
                        feeder
                            .feedWhenValidRunCommand(FeederConstants.FEED_POWER)
                            .withDeadline(new WaitCommand(3))),

                // second command(point towards target)
                drive.joystickDriveAtTarget(() -> 0, () -> 0),

                // third command, run rollers when valid
                belt.setPercentMotorOutputRunCommand(
                    BeltConstants.FEED_POWER, () -> feeder.getTargetSpeed() > 0),

                // fourth, start shooter
                shooter.setAutomaticCommandRun(),
                intake.intakeRetractWhileShooting(() -> feeder.getTargetSpeed() > 0))

            // at the very end stop the shooter, rollers, and belt
            .andThen(shooter.setManualSpeedCommand(0))
            .andThen(feeder.setPercentMotorCommand(0))
            .andThen(belt.setPercentMotorOutputCommand(0)));

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

    new EventTrigger("intakeStartEvent")
        .onTrue(
            new InstantCommand(
                () -> intake.setRollersVoltage(Constants.IntakeConstants.INTAKE_VOLTS)));
    new EventTrigger("intakeStopEvent")
        .onTrue(new InstantCommand(() -> intake.setRollersVoltage(0)));

    new EventTrigger("intakeDownEvent")
        .onTrue(new InstantCommand(() -> intake.setPosition(IntakeConstants.INTAKE_DOWN_POSITION)));

    new EventTrigger("intakeClearEvent")
        .onTrue(
            new InstantCommand(() -> intake.setPosition(IntakeConstants.INTAKE_CLEAR_POSITION)));

    new EventTrigger("shootOnMove")
        // makes the drive rotate correctly
        .onTrue(drive.overrideRotationCommand())
        .onFalse(
            drive
                .removeRotationOverrideCommand()

                // stop the mechanisms
                .andThen(shooter.setManualSpeedCommand(0))
                .andThen(feeder.setPercentMotorCommand(0))
                .andThen(belt.setPercentMotorOutputCommand(0)))

        // starts shooter + feeder + belts
        .whileTrue(
            Commands.parallel(
                new RunCommand(
                    () -> {
                      shooter.setAutomaticSpeed(1);
                    }),
                feeder.feedWhenValidRunCommandAutoEvent(FeederConstants.FEED_POWER),
                belt.setPercentMotorOutputRunCommandAutoEvent(
                    1, () -> feeder.getTargetSpeed() > 0)));

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

    new EventTrigger("reverseFeederEvent")
        .onTrue(
            new InstantCommand(
                () -> feeder.setPercentMotorOutput(FeederConstants.EJECT_POWER_AUTO)));
    new EventTrigger("stopFeederEvent")
        .onTrue(new InstantCommand(() -> feeder.setPercentMotorOutput(0)));
  }

  public Command shootAfterPathingCommand(Command pathingCommand) {
    return new ParallelCommandGroup(
        pathingCommand.andThen(
            Commands.parallel(
                drive.joystickDriveAtTarget(controller),
                feeder.feedWhenValidRunCommand(FeederConstants.FEED_POWER),
                belt.setPercentMotorOutputRunCommand(1, () -> feeder.isValidToFeed()),
                intake.intakeRetractWhileShooting(() -> feeder.isValidToFeed()))),
        shooter.setAutomaticCommandRun());
  }

  public PathPlannerPath getPath(String pathName) {
    PathPlannerPath path = null;

    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception exception) {
      System.out.println("unable to get path");
      exception.printStackTrace();
    }

    return path;
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
