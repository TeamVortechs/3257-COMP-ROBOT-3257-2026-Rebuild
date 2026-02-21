// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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

  public class DriveConstants {

    private static final InterpolatingDoubleTreeMap AIRTIME_MAP = new InterpolatingDoubleTreeMap();

    private static void characterizeAirtimeMap() {
      AIRTIME_MAP.put(0.0, 0.0);
    }

    public static final double SHOOTER_ROTATION_MANAGER_LOGGING_FREQUENCY =
        Constants.LOW_PRIORITY_FREQUENCY_HZ;

    public static final double FREQUENCY_UPDATE_ACC =
        20.00; // how many times per sec should we log accelerometer
    public static final double transKp = 2;
    public static final double transKi = 0;
    public static final double transKd = 0;

    public static final double transTopSpeed = 1.5;
    public static final double transAccMax = 2;

    // rot const
    public static final double rotKp = 1.7;
    public static final double rotKi = 0;
    public static final double rotKd = 0;

    public static final double rotTopSpeed = 100;
    public static final double rotAccMax = 110;

    // tolerances
    public static final double rotationTolerance = 0.1;
    public static final double translationTolerance = 0.01;

    public static final ProfiledPIDController ANGLE_CONTROLLER;

    public static final double ORIENTATION_TOLERANCE = .1;
    // the time it takes between feeding and actual robot shoot. This is used to lead the robot
    // pose. Should be about 0.08 - 0.18 s
    public static final double KRELEASE_POSE_PREDICTION_SEC = 0; // to change .5

    // we should test by looking at values. this can also be a distance lookup table. This corrects
    // for robot speed by changing the target location. This constant is supposed ot emmulate fligth
    // time
    public static final double KFLIGHT_COMPENSATION_SEC(double distance) {

      double val = AIRTIME_MAP.get(distance);

      Logger.recordOutput("DriveConstants/MostRecentAirTimeEstimation", val);
      // return val;

      return 0;
    }

    // the maximum allowed difference allowed between acceleraomter and encoders before it is
    // considered skid
    public static final double SKID_THRESHOLD = 1000;

    public static final double DEADBAND = 0.1;
    public static final double ANGLE_KP = 15; //12
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public static final Translation2d CENTER_POINT = new Translation2d(8.27, 4.115);

    // find this
    public static final Pose2d GOAL_POSE_BLUE = new Pose2d(4.622, 4.03, new Rotation2d());
    public static final Pose2d GOAL_POSE_RED = new Pose2d(11.917, 4.030, new Rotation2d());

    private static List<Pose2d> PASSING_GOALS_STORAGE = null;
    private static List<String> PASSING_GOALS_NAME_STORAGE = null;

    public static boolean SWICH_PASSING_GOALS = false;

    public static final List<String> PASSING_GOALS_NAMES() {

      if (PASSING_GOALS_NAME_STORAGE == null) {
        PASSING_GOALS_NAME_STORAGE = new ArrayList<>();
        PASSING_GOALS_NAME_STORAGE.add("pose 1");
        PASSING_GOALS_NAME_STORAGE.add("pose 2");
        PASSING_GOALS_NAME_STORAGE.add("pose_3");
      }

      return PASSING_GOALS_NAME_STORAGE;
    }

    public static final List<Pose2d> PASSING_GOALS() {

      if (PASSING_GOALS_STORAGE == null) {
        PASSING_GOALS_STORAGE = new ArrayList<>();
        PASSING_GOALS_STORAGE.add(new Pose2d());
        PASSING_GOALS_STORAGE.add(new Pose2d(1, 1, new Rotation2d()));
        PASSING_GOALS_STORAGE.add(new Pose2d(2, 2, new Rotation2d()));
      }

      // add flip logic here
      // double xToFlip = 5;
      // double yToFlip = 5;
      // double x;
      // double y;
      if (SWICH_PASSING_GOALS) {
        for (int i = 0; i < PASSING_GOALS_STORAGE.size(); i++) {
          PASSING_GOALS_STORAGE.set(
              i,
              PASSING_GOALS_STORAGE.get(i).rotateAround(CENTER_POINT, Rotation2d.fromDegrees(180)));
        }
        SWICH_PASSING_GOALS = false;
      }

      return PASSING_GOALS_STORAGE;
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

    public static final double K_JOYSTICK_WHEN_SHOOTING = 1;
    public static final double K_JOYSTICK_WHEN_PASSING = 1;

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
              0.0,
              ANGLE_KD,
              new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
      ANGLE_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

      characterizeAirtimeMap();
    }
  }

  public class ShooterConstants {
    public static final double FREQUENCY_HZ = Constants.HIGH_PRIORITY_FREQUENCY_HZ;

    public static final double CURRENT_LIMIT = 40.0;

    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 5;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 3;
    public static final double TOLERANCE = 5;

    public static final int MOTOR_ID = 24;

    // this is higher rn cus it's in sim. We can model this as a linear function based on distance
    // if we're having trouble adjusting but right now I'm not cus it's a variable that mgiht not be
    // necessary

    public static final double DEFAULT_SPEED = 0; // speed shooter run at default
    // speed intake/shooter boosts to
    public static final double INTAKE_SPEED = .5;

    public static final double PERCENTAGE_OF_DISTANCE_WHEN_CHARGING = 0.6;

    // the time that the feeder waits before shooting once it is valis

    // CHANGE !!
    public static final double KS = 0.0;
    public static final double KV = 0.13727;
    public static final double KP = 0.16011;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KA = 0.050592;

    public static final TalonFXConfiguration CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;
    public static final ClosedLoopRampsConfigs CLOSE_LOOP_RAMP_CONFIG;

    static {
      CONFIG = new TalonFXConfiguration();
      SLOT0CONFIGS = new Slot0Configs();
      CLOSE_LOOP_RAMP_CONFIG = new ClosedLoopRampsConfigs();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      SLOT0CONFIGS.kS = Constants.ShooterConstants.KS;
      SLOT0CONFIGS.kV = Constants.ShooterConstants.KV;
      SLOT0CONFIGS.kP = Constants.ShooterConstants.KP;
      SLOT0CONFIGS.kI = Constants.ShooterConstants.KI;
      SLOT0CONFIGS.kD = Constants.ShooterConstants.KD;
      SLOT0CONFIGS.kA = Constants.ShooterConstants.KA;

      var motionMagicConfigs = CONFIG.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
      motionMagicConfigs.MotionMagicAcceleration =
          20; // Target acceleration of 160 rps/s (0.5 seconds)
      motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    }
  }

  // copied directly from ShooterConstants
  public class FeederConstants {

    public static final double VALIDITY_LOGGING_FREQUENCY_HERTZ = LOW_PRIORITY_FREQUENCY_HZ;
    public static final double SUBSYSTEM_LOGGING_FREQUENCY_HERTZ = MEDIUM_PRIORITY_FREQUENCY_HZ;

    public static final double VALIDITY_DEBOUNCE_TIME_SEC = 0.2;

    public static final double CURRENT_LIMIT = 40.0;
    public static final double SIM_TOLERANCE = 0.5;

    public static final double FEEDER_ID = 23;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double POSITION_TOLERANCE = 0.1;

    // not real
    public static final int MOTOR_ID = 23;

    public static final double FEED_POWER = 1;

    public static final Slot0Configs SLOT0CONFIGS;
    public static final TalonFXConfiguration CONFIG;

    static {
      SLOT0CONFIGS = new Slot0Configs();
      CONFIG = new TalonFXConfiguration();

      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.FeederConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
  }

  // copied off feeder constants
  // copied directly from ShooterConstants
  public class BeltConstants {

    public static double FREQUENCY_HZ = Constants.LOW_PRIORITY_FREQUENCY_HZ;

    public static final double CURRENT_LIMIT = 40.0;

    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;

    // not real
    public static final int ID = 6;

    public static final double FEED_POWER = 0.1;

    public static final double TOLERANCE = 0.1;

    public static final TalonFXConfiguration CONFIG;

    static {
      CONFIG = new TalonFXConfiguration();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.BeltConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
  }

  // copied directly from BeltConstants
  public class ClimbConstants {
    public static final double FREQUENCY_HZ = Constants.VERY_LOW_PRIORITY_FREQUENCY_HZ;

    public static final double CURRENT_LIMIT = 40.0;
    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.1;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 0.25;

    public static final double FEED_POWER = 0.1;

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
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.ClimbConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;

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

    // dummy values for now
    public static final double MAX_TARGET_SPEED = 100;
    public static final double MAX_MANUAL_SPEED = 100;
    public static final double POS_TOLERANCE = 0.2;

    public static final double POSITION_TOLERANCE = 0.1;

    public static final double MAX_POSITION = 1;
    public static final double MIN_POSITION = 0;

    // CHANGE !!
    public static final double KS = 0.25;
    public static final double KV = 0.12;
    public static final double KA = 0.01;
    public static final double KP = 12;
    public static final double KI = 0;
    public static final double KD = 0.1;

    public static final double INTAKE_VOLTS = 8;
    public static final double INTAKE_POSITION = 0.5;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 3;
    public static final double MOTION_MAGIC_ACCELERATION = 2.5;
    public static final double MOTION_MAGIC_JERK = 10;

    public static final double RAMP_RATE_VOLTS_ROLLER_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_ROLLER_SYSID = 1;

    public static final int ROLLER_ID = 21;
    public static final int POSITION_ID = 22;

    public static final double ROLLER_STALLED_VOLTS = 20.0;
    // lower cus this has hardstops
    public static final double RAMP_RATE_VOLTS_POSITION_SYSID = 0.1;
    public static final double DYNAMIC_STEP_VOLTS_POSITION_SYSID = 0.25;
    public static final TalonFXConfiguration ROLLER_CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;

    public static final int INTAKE_ROLLER_MOTOR_ID = 21;
    public static final int INTAKE_POSITION_MOTOR_ID = 22;

    static {
      SLOT0CONFIGS = new Slot0Configs();
      SLOT0CONFIGS.kS = Constants.IntakeConstants.KS;
      SLOT0CONFIGS.kV = Constants.IntakeConstants.KV;
      SLOT0CONFIGS.kA = Constants.IntakeConstants.KA;
      SLOT0CONFIGS.kP = Constants.IntakeConstants.KP;
      SLOT0CONFIGS.kI = Constants.IntakeConstants.KI;
      SLOT0CONFIGS.kD = Constants.IntakeConstants.KD;

      ROLLER_CONFIG = new TalonFXConfiguration();
      ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.ClimbConstants.CURRENT_LIMIT;
      ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
  }
}
