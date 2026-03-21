// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.util.SmartConstant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURR_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public static final double FREQUENCY_HZ = 50;

  public static final double HIGH_PRIORITY_FREQUENCY_HZ = 50;
  public static final double MEDIUM_PRIORITY_FREQUENCY_HZ = 25;
  public static final double LOW_PRIORITY_FREQUENCY_HZ = 10;
  public static final double VERY_LOW_PRIORITY_FREQUENCY_HZ = 4;

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

    private static final InterpolatingDoubleTreeMap AIRTIME_MAP = new InterpolatingDoubleTreeMap();

    private static void characterizeAirtimeMap() {
      // bad balue
      AIRTIME_MAP.put(1.63, 0.86375);
      AIRTIME_MAP.put(2.252, 0.951);
      AIRTIME_MAP.put(2.89, 1.049);
      AIRTIME_MAP.put(3.5, 1.098);
    }

    public static final double SHOOTER_ROTATION_MANAGER_LOGGING_FREQUENCY =
        Constants.LOW_PRIORITY_FREQUENCY_HZ;

    public static final double FREQUENCY_UPDATE_ACC =
        20.00; // how many times per sec should we log accelerometer

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

    public static final SmartConstant ANGLE_KP_SETTABLE =
        new SmartConstant("DriveSettableConstantAngleKP", ANGLE_KP);
    public static final SmartConstant ANGLE_KI_SETTABLE =
        new SmartConstant("DriveSettableConstantAngleKI", ANGLE_KI);
    public static final SmartConstant ANGLE_KD_SETTABLE =
        new SmartConstant("DriveSettableConstantAngleKD", ANGLE_KD);

    public static final SmartConstant TRANS_KP_SETTABLE =
        new SmartConstant("DriveSettableConstantTransKP", TRANS_KP);
    public static final SmartConstant TRANS_KI_SETTABLE =
        new SmartConstant("DriveSettableConstantTransKI", TRANS_KI);
    public static final SmartConstant TRANS_KD_SETTABLE =
        new SmartConstant("DriveSettableConstantTransKD", TRANS_KD);

    public static Command remakeAnglePIDController() {
      return new InstantCommand(
          () -> {
            ANGLE_CONTROLLER.setP(ANGLE_KP_SETTABLE.get());
            ANGLE_CONTROLLER.setI(ANGLE_KI_SETTABLE.get());
            ANGLE_CONTROLLER.setD(ANGLE_KD_SETTABLE.get());
          });
    }

    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double ANGLE_MAX_VELOCITY = 8.0;

    public static final double ORIENTATION_TOLERANCE = 0.2;

    // this gets made with the other constants
    public static final ProfiledPIDController ANGLE_CONTROLLER;

    // the time it takes between feeding and actual robot shoot. This is used to lead the robot
    // pose. Should be about 0.08 - 0.18 s
    public static final double KRELEASE_POSE_PREDICTION_SEC = 0; // to change .5

    public static final double K_JOYSTICK_WHEN_SHOOTING = 1;
    public static final double K_JOYSTICK_WHEN_PASSING = 1;
    public static final double K_JOYSTICK_ROTATION = 0.7;
    public static final double K_JOYSTICK_TRANSLATION = 1;

    // y position that splits this in half

    // we should test by looking at values. this can also be a distance lookup table. This corrects
    // for robot speed by changing the target location. This constant is supposed ot emmulate fligth
    // time
    public static final double getTimeInAir(double distance) {

      double val = AIRTIME_MAP.get(distance);

      // realistic for a midpoint shot
      return 0.9;
    }

    // tolerance for the shoot on move binary search. TS IS NOT HTE DRIVETRAIN MOVE TO ANGLE
    // TOLERANCE
    public static final double SHOOT_ON_MOVE_TOLERANCE = 0.05;

    // the maximum allowed difference allowed between acceleraomter and encoders before it is
    // considered skid
    public static final double SKID_THRESHOLD = 1000;

    // SHOOT ON MOVE CONSTATNS.
    // used for auto teargetting

    // idrk what this does but we should never to use/change these
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public static final Translation2d CENTER_POINT = new Translation2d(8.27, 4.115);

    // this is the rotation the drive will turn to when travelling over the bumpers, depending on
    // what side of the field(red or blue)
    // this is optimal bc it's smoother going at an angle rather than straight in
    public static final double RED_SIDE_DEGREES = 135;
    public static final double BLUE_SIDE_DEGREES = 45;
    // find this
    public static final Pose2d GOAL_POSE_BLUE = new Pose2d(4.622, 4.03, new Rotation2d());
    public static final Pose2d GOAL_POSE_RED = new Pose2d(11.917, 4.030, new Rotation2d());

    private static List<Pose2d> PASSING_GOALS_STORAGE = null;
    private static List<String> PASSING_GOALS_NAME_STORAGE = null;

    public static final List<String> PASSING_GOALS_NAMES() {

      if (PASSING_GOALS_NAME_STORAGE == null) {
        PASSING_GOALS_NAME_STORAGE = new ArrayList<>();
        PASSING_GOALS_NAME_STORAGE.add("Back left");
        PASSING_GOALS_NAME_STORAGE.add("Back right");
      }

      return PASSING_GOALS_NAME_STORAGE;
    }

    public static final int GET_PASSING_GOALS() {
      GET_PASSING_GOAL(0);
      return PASSING_GOALS_STORAGE.size();
    }

    public static final Pose2d GET_PASSING_GOAL(int index) {
      Pose2d returnPose;

      if (PASSING_GOALS_STORAGE == null) {
        PASSING_GOALS_STORAGE = new ArrayList<>();
        PASSING_GOALS_STORAGE.add(new Pose2d(3.6, 6.0, new Rotation2d()));
        PASSING_GOALS_STORAGE.add(new Pose2d(3.6, 2.0, new Rotation2d()));
      }

      // if the alliance is verifiably red,
      if (DriverStation.getAlliance() != null
          && DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red) {

        // get the opposite passing pose, then rotate the whole thing around 180 degrees
        // this should mimic flipping it over an axis
        returnPose =
            PASSING_GOALS_STORAGE
                // next index mod 2
                // this wraps index = 1 + 1 back to index = 0
                .get((index + 1) % 2)
                .rotateAround(CENTER_POINT, Rotation2d.fromDegrees(180));
      } else {
        returnPose = PASSING_GOALS_STORAGE.get(index);
      }

      return returnPose;
    }

    // this is ugly but all it does is return target pose based on the team
    public static final Supplier<Pose2d> GOAL_POSE =
        () -> {
          if (DriverStation.getAlliance().isEmpty()) {
            return new Pose2d();
          }

          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return GOAL_POSE_BLUE;
          } else {
            return GOAL_POSE_RED;
          }
        };

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

      characterizeAirtimeMap();
    }
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

    // this is higher rn cus it's in sim. We can model this as a linear function based on distance
    // if we're having trouble adjusting but right now I'm not cus it's a variable that mgiht not be
    // necessary

    public static final double DEFAULT_SPEED = 0; // speed shooter run at default
    // speed intake/shooter boosts to
    public static final double INTAKE_SPEED = .5;

    public static final double PERCENTAGE_OF_DISTANCE_WHEN_CHARGING = 0.6;

    public static final SmartConstant SHOOTER_TEST_SPEED =
        new SmartConstant("shooter test speed", 70);

    // the time that the feeder waits before shooting once it is valis

    // CHANGE !
    public static final double KS = 0.1021;
    public static final double KV = 0.12071;
    public static final double KP = 0.5; // 10991
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KA = 0.016241;

    public static final TalonFXConfiguration CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;
    public static final ClosedLoopRampsConfigs CLOSE_LOOP_RAMP_CONFIG;

    static {
      CONFIG = new TalonFXConfiguration();
      SLOT0CONFIGS = new Slot0Configs();
      CLOSE_LOOP_RAMP_CONFIG = new ClosedLoopRampsConfigs();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double POSITION_TOLERANCE = 0.1;

    // not real
    public static final int MOTOR_ID = 23;

    public static final double FEED_POWER = 1;
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

    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;

    // not real
    public static final int ID = 25;

    public static final double FEED_POWER = 0.6;
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

  // copied directly from BeltConstants
  public class ClimbConstants {
    public static final double FREQUENCY_HZ = Constants.VERY_LOW_PRIORITY_FREQUENCY_HZ;

    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.1;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 0.25;

    public static final double CLIMB_UP_VOLTS = 0.1;
    public static final double CLIMB_DOWN_VOLTS = -0.1;

    public static final double MIN_POSITION_LEFT = 0;
    public static final double MAX_POSITION_LEFT = 1;

    public static final double MIN_POSITION_RIGHT = 0;
    public static final double MAX_POSITION_RIGHT = 1;

    // CHANGE !!
    public static final double KS = 0.1;
    public static final double KV = 0.12;
    public static final double KP = 0.11;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final int servoChannel = 9;

    public static final int SimulationID = 1;

    public static final TalonFXConfiguration CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;

    public static final int RIGHT_ID = 26;
    public static final int LEFT_ID = 27;
    public static final int SERVO_ID = 28;

    static {
      CONFIG = new TalonFXConfiguration();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit =
          Constants.CurrentLimitConstants.SUPPLY_CURRENT_LIMIT_CLIMB;
      CONFIG.CurrentLimits.StatorCurrentLimit =
          Constants.CurrentLimitConstants.STATOR_CURRENT_LIMIT_CLIMB;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

      SLOT0CONFIGS = new Slot0Configs();
      SLOT0CONFIGS.kS = Constants.ClimbConstants.KS;
      SLOT0CONFIGS.kV = Constants.ClimbConstants.KV;
      SLOT0CONFIGS.kP = Constants.ClimbConstants.KP;
      SLOT0CONFIGS.kI = Constants.ClimbConstants.KI;
      SLOT0CONFIGS.kD = Constants.ClimbConstants.KD;
    }

    public static final double SERVO_OPEN = 0;
    public static final double SERVO_CLOSED = 10; // in degrees TODO: change
  }

  public class IntakeConstants {
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

    public static final double INTAKE_DOWN_POSITION = 0.846;
    public static final double INTAKE_HALFWAY_UP_POSITION = 0.380; // 0.6
    public static final double DEEP_INTAKE_UP_POSITION = 0.2;
    public static final double INTAKE_HALFWAY_LOWER_POSITION = 0.5;
    public static final double INTAKE_CLEAR_POSITION = 0.75;
    public static final double INTAKE_UP_POSITION = 0.156; // 0.156
    // .-0.062

    public static final double CLAMP_MAX_VOLTS = 3;
    public static final double POSITION_THRESHOLD_STOP = 0.2;

    // CHANGE !!
    public static final double KS = 0;
    public static final double KV = 0.2;
    public static final double KA = 0;
    public static final double KP = 20;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KG = 0.5;

    public static final double ROLLER_GOING_DOWN_VOLTS = -12;
    public static final double ROLLER_GOING_UP_VOLTS = 4.5;
    public static final double INTAKE_VOLTS = 12;
    public static final double EJECT_VOLTS = -8;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 10;
    public static final double MOTION_MAGIC_ACCELERATION = 5;
    public static final double MOTION_MAGIC_JERK = 100000000.0;
    // for slowing down the intake when attempting to close while firing
    public static final Time WAIT_TIME_TO_PULL_INTAKE = Seconds.of(2);
    public static final double MOTION_MAGIC_SLOWED_VELOCITY = 0.25; // originally 1.5
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

    public static final int INTAKE_ROLLER_MOTOR_ID = 21;
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
}
