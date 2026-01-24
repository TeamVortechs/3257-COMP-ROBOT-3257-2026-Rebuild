// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class ShooterConstants {
    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double TOLERANCE = 0.1;

    public static final int ID = 1;

    // this is higher rn cus it's in sim. We can model this as a linear function based on distance
    // if we're having trouble adjusting but right now I'm not cus it's a variable that mgiht not be
    // necessary
    public static final double ORIENTATION_TOLERANCE = .1;

    public static final double DEFAULT_SPEED = .25; // speed intake/shooter run at default
    // speed intake/shooter boosts to
    public static final double INTAKE_SPEED = .5;

    public static final double X_POSE_TO_CHARGE = 5.5;
    public static final double PERCENTAGE_OF_DISTANCE_WHEN_CHARGING = 0.6;

    // the time it takes between feeding and actual robot shoot. This is used to lead the robot
    // pose. Should be about 0.08 - 0.18 s
    public static final double KRELEASE_POSE_PREDICTION_SEC = 0.08;

    // we should test by looking at values. this can also be a distance lookup table. This corrects
    // for robot speed by changing the target location. This constant is supposed ot emmulate fligth
    // time
    public static final double KFLIGHT_COMPENSATION_SEC = 0.2;

    public static final double K_JOYSTICK_WHEN_SHOOTING = 0.5;

    // the time that the feeder waits before shooting once it is valis
    public static final double VALIDITY_DEBOUNCE_TIME_SEC = 0.2;

    // the maximum allowed difference allowed between acceleraomter and encoders before it is
    // considered skid
    public static final double SKID_THRESHOLD = 0.5;
  }

  // copied directly from ShooterConstants
  public class FeederConstants {
    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double TOLERANCE = 0.1;

    // not real
    public static final int ID = 0;

    public static final double FEED_POWER = 0.1;
  }

  // copied off feeder constants
  // copied directly from ShooterConstants
  public class BeltConstants {

    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;

    // not real
    public static final int ID = 6;

    public static final double FEED_POWER = 0.1;
  }

  // copied directly from BeltConstants
  public class ClimbConstants {

    // used in Belt.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;

    // not real
    public static final int ID = 6;

    public static final double FEED_POWER = 0.1;

    public static final double MIN_POSITION_LEFT = 0;
    public static final double MAX_POSITION_LEFT = 0;

    public static final double MIN_POSITION_RIGHT = 0;
    public static final double MAX_POSITION_RIGHT = 0;
  }
}
