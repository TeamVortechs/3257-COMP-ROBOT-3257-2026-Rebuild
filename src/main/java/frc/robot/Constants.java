// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.filtering.DeadbandDriveInputFilter;
import frc.robot.subsystems.drive.filtering.DriveInputFilter;
import frc.robot.util.SmartConstant;
import frc.robot.util.VortechsUtil;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final CANBus MECHANISM_CANBUS = new CANBus("Mech - Canivore");

  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURR_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public static final double FREQUENCY_HZ = 50;

  public static final double HIGH_PRIORITY_FREQUENCY_HZ = 50;
  public static final double MEDIUM_PRIORITY_FREQUENCY_HZ = 25;
  public static final double LOW_PRIORITY_FREQUENCY_HZ = 10;
  public static final double VERY_LOW_PRIORITY_FREQUENCY_HZ = 4;

  public static final Supplier<Alliance> ALLIANCE =
      () -> {
        if (DriverStation.getAlliance() == null || DriverStation.getAlliance().isEmpty()) {
          return Alliance.Blue;
        }

        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          return Alliance.Blue;
        } else {
          return Alliance.Red;
        }
      };

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class CurrentLimitConstants {
    // public static final double SUPPLY_CURRENT_LIMIT_DRIVE = 40.0;
    // public static final double STATOR_CURRENT_LIMIT_DRIVE = 40.0;

    // Drive doesn't have these limits here. That's because it has limit constants from another
    // file.
    // I decided not to mess with all that... so the limit is at 120. Check ModuleIOTalonFX.java for
    // more info

    public static final double SUPPLY_CURRENT_LIMIT_SHOOTER = 500;
    public static final double STATOR_CURRENT_LIMIT_SHOOTER = 100000;

    public static final double SUPPLY_CURRENT_LIMIT_FEEDER = 40.0;
    public static final double STATOR_CURRENT_LIMIT_FEEDER = 120;

    public static final double SUPPLY_CURRENT_LIMIT_BELT = 40.0;
    public static final double STATOR_CURRENT_LIMIT_BELT = 40.0;

    public static final double SUPPLY_CURRENT_LIMIT_CLIMB = 40.0;
    public static final double STATOR_CURRENT_LIMIT_CLIMB = 40.0;

    public static final double SUPPLY_CURRENT_LIMIT_INTAKE_POSITION = 300;
    public static final double STATOR_CURRENT_LIMIT_INTAKE_POSITION = 10000;

    public static final double SUPPLY_CURRENT_LIMIT_INTAKE_ROLLER = 300;
    public static final double STATOR_CURRENT_LIMIT_INTAKE_ROLLER = 80.0;
  }

  public class DriveConstants {

    // pathplanner constants
    public static final double ROBOT_WEIGHT = 58.060;
    public static final double ROBOT_MOI = 7.218;
    public static final double WHEEL_COF = 1.2;

    // pid constants
    public static final double TRANS_KP = 5;
    public static final double TRANS_KI = 0;
    public static final double TRANS_KD = 0;

    public static final double TRANS_TOP_SPEED = 1.5;
    public static final double TRANS_ACC_MAX = 2;

    public static final double TRANS_TOLERANCE = 0.01;

    // rot const, used for moving to setpoint/auto targetting
    public static final double ANGLE_KP = 2;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_DEADBAND = 0.1;

    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double ANGLE_MAX_VELOCITY = 8.0;

    public static final double ORIENTATION_TOLERANCE = 0.2;

    // this gets made with the other constants
    public static final ProfiledPIDController ANGLE_CONTROLLER;

    public static final double K_JOYSTICK_WHEN_SHOOTING = 1;
    public static final double K_JOYSTICK_WHEN_PASSING = 1;
    public static final double K_JOYSTICK_ROTATION = 0.7;
    public static final double K_JOYSTICK_TRANSLATION = 1;

    // SHOOT ON MOVE CONSTATNS.
    // used for auto teargetting

    public static final Translation2d CENTER_POINT = new Translation2d(8.27, 4.115);

    public static double MAX_LINEAR_SPEED_METERS_PER_SECOND =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    public static double MAX_ANGULAR_SPEED_RAD_PER_SEC() {
      return MAX_LINEAR_SPEED_METERS_PER_SECOND / DRIVE_BASE_RADIUS;
    }

    // control req stuff:
    public static SwerveRequest.FieldCentric DRIVE_CONTROL_REQ =
        new FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

    public static final Supplier<Pose2d> BUMPER_SHOOT_POSE_LEFT =
        VortechsUtil.AllianceBasedPose(
            new Pose2d(3.536, 6.483, Rotation2d.fromDegrees(-66.648)),
            new Pose2d(13.004, 1.512, Rotation2d.fromDegrees(-114.615)));
    public static final Supplier<Pose2d> BUMPER_SHOOT_POSE_RIGHT =
        VortechsUtil.AllianceBasedPose(
            new Pose2d(3.536, 1.566, Rotation2d.fromDegrees(66.648)),
            new Pose2d(13.004, 6.504, Rotation2d.fromDegrees(-114.615)));

    // POSE STUFF
    public static final Supplier<Pose2d> CLIMB_SHOOT_POSE_RIGHT =
        VortechsUtil.AllianceBasedPose(
            new Pose2d(1.277, 2.889, Rotation2d.fromDegrees(19.479)),
            new Pose2d(15.242, 5.170, Rotation2d.fromDegrees(-161.34)));
    public static final Supplier<Pose2d> CLIMB_SHOOT_POSE_LEFT =
        VortechsUtil.AllianceBasedPose(
            new Pose2d(1.201, 4.621, Rotation2d.fromDegrees(-9.11)),
            new Pose2d(15.350, 3.459, Rotation2d.fromDegrees(170.567)));

    public static final Pose2d PASSING_POSE_UP_BLUE = new Pose2d(2.5, 6, new Rotation2d());
    public static final Pose2d PASSING_POSE_DOWN_BLUE = new Pose2d(2.5, 2, new Rotation2d());

    public static final Pose2d PASSING_POSE_UP_RED = new Pose2d(14, 6, new Rotation2d());
    public static final Pose2d PASSING_POSE_DOWN_RED = new Pose2d(14, 2, new Rotation2d());

    public static final Supplier<Pose2d> PASSING_POSE_DOWN =
        VortechsUtil.AllianceBasedPose(PASSING_POSE_DOWN_BLUE, PASSING_POSE_DOWN_RED);
    public static final Supplier<Pose2d> PASSING_POSE_UP =
        VortechsUtil.AllianceBasedPose(PASSING_POSE_UP_BLUE, PASSING_POSE_UP_RED);

    // find this
    public static final Pose2d GOAL_POSE_BLUE = new Pose2d(4.622, 4.03, new Rotation2d());
    public static final Pose2d GOAL_POSE_RED = new Pose2d(11.917, 4.030, new Rotation2d());

    public static final Supplier<Pose2d> GOAL_POSE =
        VortechsUtil.AllianceBasedPose(GOAL_POSE_BLUE, GOAL_POSE_RED);

    // the zone where we choose to more agressively charge the shooter
    public static final double X_POSE_TO_CHARGE = 5.5;
    public static final double X_POSE_TO_PASS = 5.5;
    public static final double HALF_MAP_Y = 4.059;

    // from our library
    public static final double ODOMETRY_FREQUENCY =
        TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Math.hypot(
                    TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Math.hypot(
                    TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    static {
      ANGLE_CONTROLLER =
          new ProfiledPIDController(
              ANGLE_KP,
              ANGLE_KI,
              ANGLE_KD,
              new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
      ANGLE_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static PPHolonomicDriveController PATHPLANNER_CONTROLLER =
        new PPHolonomicDriveController(
            new PIDConstants(
                DriveConstants.TRANS_KP, DriveConstants.TRANS_KI, DriveConstants.TRANS_KD),
            new PIDConstants(
                DriveConstants.ANGLE_KP, DriveConstants.ANGLE_KI, DriveConstants.ANGLE_KD));

    public static final RobotConfig PP_CONFIG =
        new RobotConfig(
            ROBOT_WEIGHT,
            ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            getModuleTranslations());

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      };
    }

    public static final DriveInputFilter DRIVE_INPUT_FILTER =
        new DeadbandDriveInputFilter(
            0.1 * MAX_LINEAR_SPEED_METERS_PER_SECOND,
            0.1 * MAX_ANGULAR_SPEED_RAD_PER_SEC(),
            MAX_LINEAR_SPEED_METERS_PER_SECOND,
            MAX_ANGULAR_SPEED_RAD_PER_SEC());
  }

  public class ShooterConstants {
    public static final double FREQUENCY_HZ = Constants.HIGH_PRIORITY_FREQUENCY_HZ;

    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 2;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 8;
    public static final double TOLERANCE = 4;

    public static final int MOTOR_ID = 24;
    public static final int FOLLOWER_MOTOR_ID = 28;
    public static final int FOLLOWER_2_MOTOR_ID = 27;

    // this is higher rn cus it's in sim. We can model this as a linear function based on distance
    // if we're having trouble adjusting but right now I'm not cus it's a variable that mgiht not be
    // necessary

    public static final double DEFAULT_SPEED = 0; // speed shooter run at default

    public static final SmartConstant SHOOTER_TEST_SPEED =
        new SmartConstant("shooter test speed", 70);

    // the time that the feeder waits before shooting once it is valis

    // CHANGE !
    public static final double KS = 0;
    public static final double KV = 0.12052;
    public static final double KP = 0.5; // 11339
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KA = 0.026169;

    public static final TalonFXConfiguration CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;
    public static final ClosedLoopRampsConfigs CLOSE_LOOP_RAMP_CONFIG;

    static {
      CONFIG = new TalonFXConfiguration();
      SLOT0CONFIGS = new Slot0Configs();
      CLOSE_LOOP_RAMP_CONFIG = new ClosedLoopRampsConfigs();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit =
          Constants.CurrentLimitConstants.SUPPLY_CURRENT_LIMIT_SHOOTER;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CONFIG.CurrentLimits.StatorCurrentLimit =
          Constants.CurrentLimitConstants.STATOR_CURRENT_LIMIT_SHOOTER;
      CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
      SLOT0CONFIGS.kS = Constants.ShooterConstants.KS;
      SLOT0CONFIGS.kV = Constants.ShooterConstants.KV;
      SLOT0CONFIGS.kP = Constants.ShooterConstants.KP;
      SLOT0CONFIGS.kI = Constants.ShooterConstants.KI;
      SLOT0CONFIGS.kD = Constants.ShooterConstants.KD;
      SLOT0CONFIGS.kA = Constants.ShooterConstants.KA;

      var motionMagicConfigs = CONFIG.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
      motionMagicConfigs.MotionMagicAcceleration =
          70; // Target acceleration of 160 rps/s (0.5 seconds)
      motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }
  }

  // copied directly from ShooterConstants
  public class FeederConstants {

    public static final double VALIDITY_LOGGING_FREQUENCY_HERTZ = LOW_PRIORITY_FREQUENCY_HZ;
    public static final double SUBSYSTEM_LOGGING_FREQUENCY_HERTZ = MEDIUM_PRIORITY_FREQUENCY_HZ;

    public static final double VALIDITY_DEBOUNCE_TIME_SEC = 0.2;

    public static final double SIM_TOLERANCE = 0.5;

    public static final double FEEDER_ID = 23;

    // not real
    public static final int MOTOR_ID = 23;

    public static final double FEED_POWER = 0.7;
    public static final double EJECT_POWER_AUTO = -1;

    public static final Slot0Configs SLOT0CONFIGS;
    public static final TalonFXConfiguration CONFIG;

    static {
      SLOT0CONFIGS = new Slot0Configs();
      CONFIG = new TalonFXConfiguration();

      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit =
          Constants.CurrentLimitConstants.SUPPLY_CURRENT_LIMIT_FEEDER;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CONFIG.CurrentLimits.StatorCurrentLimit =
          Constants.CurrentLimitConstants.STATOR_CURRENT_LIMIT_FEEDER;
      CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    }
  }

  // copied off feeder constants
  // copied directly from ShooterConstants
  public class BeltConstants {

    public static double FREQUENCY_HZ = Constants.LOW_PRIORITY_FREQUENCY_HZ;

    // not real
    public static final int ID = 25;

    public static final double FEED_POWER = 1;
    public static final double INTAKE_POWER = 0;
    public static final double DEFAULT_POWER = 0;
    public static final double EJECT_POWER = -1;

    public static final double TOLERANCE = 0.1;

    public static final TalonFXConfiguration CONFIG;

    static {
      CONFIG = new TalonFXConfiguration();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit =
          Constants.CurrentLimitConstants.SUPPLY_CURRENT_LIMIT_BELT;
      CONFIG.CurrentLimits.StatorCurrentLimit =
          Constants.CurrentLimitConstants.STATOR_CURRENT_LIMIT_BELT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    }
  }

  public class IntakeConstants {

    // JAM VALUES: if we exceed all these values at the same time, we're prob in a jam

    // these all depend on the motor. right now, these are just generic values, will update after
    // research
    public static final double ROLLER_JAM_CURRENT_AMPS = 40.0;
    public static final double ROLLER_JAM_VELOCITY = 2.0;
    public static final double POSITION_JAM_CURRENT_AMPS = 35.0;
    // Minimum voltage applied on a motor for there to be considered a jam(unit: volts)
    public static final double MIN_VOLTAGE_APPLIED = 2.0;

    public static final double FREQUENCY_HZ = Constants.LOW_PRIORITY_FREQUENCY_HZ;

    public static final double TIME_TO_WAIT_BEFORE_RETRACT_ON_SHOOT = 1.5;

    // dummy values for now
    public static final double MAX_TARGET_SPEED = 100;
    public static final double MAX_MANUAL_SPEED = 100;
    public static final double MAX_MANUAL_VOLTS = 2;
    // public static final double POSx_TOLERANCE = 0.2;

    public static final double POSITION_TOLERANCE = 0.1;

    // 2
    // 1.29
    public static final double MIN_POSITION = -10000; // o.373535
    public static final double MAX_POSITION = 1000; // can also do 0.36

    public static final double INTAKE_DOWN_POSITION = 0.711;
    public static final double INTAKE_HALFWAY_UP_POSITION = 0.47; // 0.6

    // used in robot container for the oscilation operator  commands
    public static final double DEEP_INTAKE_UP_POSITION = 0.4;
    public static final double INTAKE_HALFWAY_LOWER_POSITION = 0.5;

    public static final double INTAKE_CLEAR_POSITION = 0.75;
    public static final double INTAKE_UP_POSITION = 0.35; // 0.156

    public static final double CLAMP_MAX_VOLTS = 3;
    public static final double POSITION_THRESHOLD_STOP = 0.2;

    // CHANGE !!
    public static final double KS = 0;
    public static final double KV = 0.2;
    public static final double KA = 0;
    public static final double KP = 100;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KG = 0.5;

    public static final double ROLLER_GOING_DOWN_VOLTS = -12;
    public static final double ROLLER_GOING_UP_VOLTS = 4.5;
    public static final double INTAKE_VOLTS = 10;
    public static final double EJECT_VOLTS = -8;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 40;
    public static final double MOTION_MAGIC_ACCELERATION = 30;
    public static final double MOTION_MAGIC_JERK = 100000000.0;
    // for slowing down the intake when attempting to close while firing
    public static final Time WAIT_TIME_TO_PULL_INTAKE = Seconds.of(2);
    public static final double MOTION_MAGIC_SLOWED_VELOCITY = 1.5; // originally 1.5
    public static final double MOTION_MAGIC_SLOWED_VELOCITY_SECOND_TIME = 1.25;
    // constants for the oscillateIntake command
    public static final Time WAIT_TIME_BETWEEN_INTAKE_OSCILLATION = Seconds.of(0.5);
    public static final double OSCILLATION_VELOCITY = 1;

    public static final double RAMP_RATE_VOLTS_ROLLER_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_ROLLER_SYSID = 1;

    public static final double ROLLER_STALLED_VOLTS = 20.0;
    // lower cus this has hardstops
    public static final double RAMP_RATE_VOLTS_POSITION_SYSID = 0.1;
    public static final double DYNAMIC_STEP_VOLTS_POSITION_SYSID = 0.25;
    public static final TalonFXConfiguration ROLLER_CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;
    public static final TalonFXConfiguration POSITION_CONFIG;
    public static final CANcoderConfiguration CANCODER_CONFIG;

    public static final int INTAKE_ROLLER_MOTOR_ID = 21;
    public static final int INTAKE_ROLLER_2_MOTOR_ID = 26;
    public static final int INTAKE_POSITION_MOTOR_ID = 22;
    public static final int INTAKE_CANCODER_ID = 29;

    public static final double CANCODER_ROTOR_TO_SENSOR_RATIO = 1; // used to be 30

    static {
      // PID constants, gravity type, static feedforward sign
      SLOT0CONFIGS = new Slot0Configs();
      SLOT0CONFIGS.kS = Constants.IntakeConstants.KS;
      SLOT0CONFIGS.kV = Constants.IntakeConstants.KV;
      SLOT0CONFIGS.kA = Constants.IntakeConstants.KA;
      SLOT0CONFIGS.kP = Constants.IntakeConstants.KP;
      SLOT0CONFIGS.kI = Constants.IntakeConstants.KI;
      SLOT0CONFIGS.kD = Constants.IntakeConstants.KD;
      SLOT0CONFIGS.kG = IntakeConstants.KG;
      SLOT0CONFIGS.GravityType = GravityTypeValue.Arm_Cosine;
      SLOT0CONFIGS.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

      // CTRE constants: brake, invert, current limits
      POSITION_CONFIG = new TalonFXConfiguration();
      POSITION_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      POSITION_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      POSITION_CONFIG.CurrentLimits.SupplyCurrentLimit =
          Constants.CurrentLimitConstants.SUPPLY_CURRENT_LIMIT_INTAKE_POSITION;
      POSITION_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      POSITION_CONFIG.CurrentLimits.StatorCurrentLimit =
          Constants.CurrentLimitConstants.STATOR_CURRENT_LIMIT_INTAKE_POSITION;
      POSITION_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

      // motion magic constants
      POSITION_CONFIG.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
      POSITION_CONFIG.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
      POSITION_CONFIG.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

      CANCODER_CONFIG = new CANcoderConfiguration();
      CANCODER_CONFIG.MagnetSensor.MagnetOffset = 0.7;
      CANCODER_CONFIG.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
      CANCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      // CTRE constants for the rollers
      ROLLER_CONFIG = new TalonFXConfiguration();
      ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimit =
          Constants.CurrentLimitConstants.SUPPLY_CURRENT_LIMIT_INTAKE_ROLLER;
      ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ROLLER_CONFIG.CurrentLimits.StatorCurrentLimit =
          Constants.CurrentLimitConstants.STATOR_CURRENT_LIMIT_INTAKE_ROLLER;
      ROLLER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    }
  }

  public class MatchTimelineConstants {
    public static final double TIMER_FREQUENCY = 1.0 / 4.0; // Also used for periodic measurement
    public static final double SHIFT_LENGTH = 25.0; // Length of each shift
    public static final double SHOOTING_BUFFER_TIME = 3.0;

    // DUMMY VALUES, update when there's time at a practice field to more realistic numbers
    public static final double SENSOR_TIME =
        .5; // Time it takes for ball to go from entering hub to roll past hub
    public static final double SHOOTING_TOLERANCE = .25;
    public static final double MIN_FLIGHT_LENGTH = 0.0;
    public static final double MAX_FLIGHT_LENGTH = 3.0;
  }
}
