// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // angles
  public static final double ARM_SCORING_ANGLE = 0.5; // set
  public static final double ARM_INTAKE_ANGLE = 11.3; // set
  public static final double ARM_MIN_ANGLE = 0.1; // set

  // controller constants
  public static final double CONTROLLER_FEEDBACK_AMOUNT = 0.2;

  // controller times
  public static final double CORAL_RELEASE_TIME = 1;

  // controller sppeds
  public static final double INTAKE_SPEED = 1;
  public static final double EJECT_SPEED = -1;
  public static final double HOLDING_SPEED = 0;

  // canrange constants
  public static final double FREE_CANRANGE_DIST = 1;
  public static final double CANRANGE_DETECTION_DISTANCE = 1;

  // arm constants
  public static final double ARM_kS = 0; // set; default 0.25
  public static final double ARM_kV = 0.25; // set; default 0.12
  public static final double ARM_kA = 0; // set; default 0.01
  public static final double ARM_kP = 0.35; // set; default Much too large
  public static final double ARM_kI = 0; // set
  public static final double ARM_kD = 0.28; // set; default 0.1
  public static final double ARM_kG = 0; // set

  public static final double ARM_SPEED_LIMIT = 6; // set; these are motionmagic constants
  public static final double ARM_ACCELERATION_LIMIT = 9; // set
  public static final double ARM_JERK_LIMIT = 0; // set
  public static final double ARM_TOLERANCE = 0.1; // set
  public static final double ARM_CURRENT_LIMIT =
      20; // set // shoudl be 65, currently one for testing
  public static final double ARM_MAX_ANGLE = 11; // set

  // var slot0Configs = armMotorConfigs.Slot0;
  // slot0Configs.kS = PWrist.kS.getValue(); // Add 0.25 V output to overcome static friction
  // slot0Configs.kV = PWrist.kV.getValue(); // A velocity target of 1 rps results in 0.12 V output
  // slot0Configs.kA = PWrist.kA.getValue(); // An acceleration of 1 rps/s requires 0.01 V output
  // slot0Configs.kP =
  //     PWrist.kP.getValue(); // A position error of 2.5 rotations results in 12 V output
  // slot0Configs.kI = PWrist.kI.getValue(); // no output for integrated error
  // slot0Configs.kD = PWrist.kD.getValue(); // A velocity error of 1 rps results in 0.1 V output

  // var motionMagicConfigs = armMotorConfigs.MotionMagic;
  // motionMagicConfigs.MotionMagicCruiseVelocity = PWrist.speedLimit.getValue();
  // motionMagicConfigs.MotionMagicAcceleration = PWrist.accelerationLimit.getValue();
  // motionMagicConfigs.MotionMagicJerk = PWrist.jerkLimit.getValue();

  // intake constants
  public static final double INTAKE_CURRENT_LIMIT =
      20; // originall at 100, need to do subsystem testing to find this. At 1 rn for testing.
  public static double INTAKE_MAX_POSITION = 0.0;
  public static double INTAKE_TOLERANCE = 0.1;

  public static boolean isFlipped() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // public class CArm {
  //   public static final double TOLERANCE = 0.1;
  //   public static final double MANUAL_MAX_SPEED = 0.1;
  //   public static final double MIN_HEIGHT = 0;
  //   public static final double MAX_HEIGHT = 5;
  //   public static final double CLEARANCE_ANGLE = 0.1;
  // }

  public class CIntake {
    // dummy values for now
    public static final double MAX_TARGET_SPEED = 100;
    public static final double MAX_MANUAL_SPEED = 100;
    public static final double SPEED_TOLERANCE = 0.2;
  }

  public class CDrivetrain {
    public static PathConstraints DEFAULT_PATH_CONSTRAINTS =
        new PathConstraints(3.5, 4, Math.PI * 3.5, Math.PI * 4);

    public static double TOLERANCE_DIST = 0.1;
    public static double TOLERANCE_ROT = 0.1;

    // PID constants for frc.robot.commands.driveCommands.PathfindToPoseCommand

    // translational const
    public static final double transKp = 2;
    public static final double transKi = 0;
    public static final double transKd = 0;

    public static final double transTopSpeed = 0.5;
    public static final double transAccMax = 0.75;

    // rot const
    public static final double rotKp = 1.7;
    public static final double rotKi = 0;
    public static final double rotKd = 0;

    public static final double rotTopSpeed = 100;
    public static final double rotAccMax = 110;

    // tolerances
    public static final double rotationTolerance = 0.1;
    public static final double translationTolerance = 0.1;

    public static final double totalDistTol = 1;
    public static final double totalRotTol = .2;
  }
}
