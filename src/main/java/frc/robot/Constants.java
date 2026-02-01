// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURR_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
  public static final double FREQUENCY_HZ = 50;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class DriveConstants {

    public static final double ORIENTATION_TOLERANCE = .1;
    // the time it takes between feeding and actual robot shoot. This is used to lead the robot
    // pose. Should be about 0.08 - 0.18 s
    public static final double KRELEASE_POSE_PREDICTION_SEC = 0;

    // we should test by looking at values. this can also be a distance lookup table. This corrects
    // for robot speed by changing the target location. This constant is supposed ot emmulate fligth
    // time
    public static final double KFLIGHT_COMPENSATION_SEC = 0;

    // the maximum allowed difference allowed between acceleraomter and encoders before it is
    // considered skid
    public static final double SKID_THRESHOLD = 5.0;

    // find this
    public static final Pose2d GOAL_POSE = new Pose2d(4.622, 4.03, new Rotation2d());

    // the zone where we choose to more agressively charge the shooter
    public static final double X_POSE_TO_CHARGE = 5.5;

    public static final double K_JOYSTICK_WHEN_SHOOTING = 0.5;

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
  }

  public class ShooterConstants {
    public static final double CURRENT_LIMIT = 40.0;

    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double TOLERANCE = 0.1;

    public static final int MOTOR_ID = 1;

    // this is higher rn cus it's in sim. We can model this as a linear function based on distance
    // if we're having trouble adjusting but right now I'm not cus it's a variable that mgiht not be
    // necessary

    public static final double DEFAULT_SPEED = 0; // speed shooter run at default
    // speed intake/shooter boosts to
    public static final double INTAKE_SPEED = .5;

    public static final double PERCENTAGE_OF_DISTANCE_WHEN_CHARGING = 0.6;

    // the time that the feeder waits before shooting once it is valis

    // CHANGE !!
    public static final double KS = 0.1;
    public static final double KV = 0.12;
    public static final double KP = 0.11;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final TalonFXConfiguration CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;

    static {
      CONFIG = new TalonFXConfiguration();
      SLOT0CONFIGS = new Slot0Configs();
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      SLOT0CONFIGS.kS = Constants.ShooterConstants.KS;
      SLOT0CONFIGS.kV = Constants.ShooterConstants.KV;
      SLOT0CONFIGS.kP = Constants.ShooterConstants.KP;
      SLOT0CONFIGS.kI = Constants.ShooterConstants.KI;
      SLOT0CONFIGS.kD = Constants.ShooterConstants.KD;
    }
  }

  // copied directly from ShooterConstants
  public class FeederConstants {

    public static final double VALIDITY_DEBOUNCE_TIME_SEC = 0.2;

    public static final double CURRENT_LIMIT = 40.0;
    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double POSITION_TOLERANCE = 0.1;

    // CHANGE !!
    public static final double KS = 0.1;
    public static final double KV = 0.12;
    public static final double KP = 0.11;
    public static final double KI = 0;
    public static final double KD = 0;

    // not real
    public static final int MOTOR_ID = 0;

    public static final double FEED_POWER = 0.1;

    public static final Slot0Configs SLOT0CONFIGS;
    public static final TalonFXConfiguration CONFIG;

    static {
      SLOT0CONFIGS = new Slot0Configs();
      CONFIG = new TalonFXConfiguration();
      SLOT0CONFIGS.kS = Constants.FeederConstants.KS;
      SLOT0CONFIGS.kV = Constants.FeederConstants.KV;
      SLOT0CONFIGS.kP = Constants.FeederConstants.KP;
      SLOT0CONFIGS.kI = Constants.FeederConstants.KI;
      SLOT0CONFIGS.kD = Constants.FeederConstants.KD;

      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.FeederConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
  }

  // copied off feeder constants
  // copied directly from ShooterConstants
  public class BeltConstants {
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
      CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.BeltConstants.CURRENT_LIMIT;
      CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
  }

  // copied directly from BeltConstants
  public class ClimbConstants {
    public static final double CURRENT_LIMIT = 40.0;
    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.1;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 0.25;

    // not real
    public static final int SERVO_ID = 6;

    public static final double FEED_POWER = 0.1;

    public static final double MIN_POSITION_LEFT = 0;
    public static final double MAX_POSITION_LEFT = 0;

    public static final double MIN_POSITION_RIGHT = 0;
    public static final double MAX_POSITION_RIGHT = 0;

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
  }

  public class IntakeConstants {
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

    public static final double INTAKE_SPEED = 0.1;
    public static final double INTAKE_POSITION = 0.5;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 3;
    public static final double MOTION_MAGIC_ACCELERATION = 2.5;
    public static final double MOTION_MAGIC_JERK = 10;

    public static final double RAMP_RATE_VOLTS_ROLLER_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_ROLLER_SYSID = 1;

    public static final double ROLLER_STALLED_VOLTS = 20.0;
    // lower cus this has hardstops
    public static final double RAMP_RATE_VOLTS_POSITION_SYSID = 0.1;
    public static final double DYNAMIC_STEP_VOLTS_POSITION_SYSID = 0.25;
    public static final TalonFXConfiguration CONFIG;
    public static final Slot0Configs SLOT0CONFIGS;

    static {
      CONFIG = new TalonFXConfiguration();
      SLOT0CONFIGS = CONFIG.Slot0;
      SLOT0CONFIGS.kS = Constants.IntakeConstants.KS;
      SLOT0CONFIGS.kV = Constants.IntakeConstants.KV;
      SLOT0CONFIGS.kA = Constants.IntakeConstants.KA;
      SLOT0CONFIGS.kP = Constants.IntakeConstants.KP;
      SLOT0CONFIGS.kI = Constants.IntakeConstants.KI;
      SLOT0CONFIGS.kD = Constants.IntakeConstants.KD;
    }
  }
}
