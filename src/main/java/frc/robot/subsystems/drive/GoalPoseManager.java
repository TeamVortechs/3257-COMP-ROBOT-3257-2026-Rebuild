package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class GoalPoseManager {
  private int indexOfPassingPose;
  private int maxIndexOfPassingPose = DriveConstants.GET_PASSING_GOALS();

  // not autologged bc we only log it on the setter
  private boolean isPassing = false;

  public GoalPoseManager() {
    // start the log
    Logger.recordOutput("GoalPoseManager/PassingPoseName", getPassingPoseName());
    Logger.recordOutput("GoalPoseManager/targetPose", getTargetPose());
    Logger.recordOutput("GoalPoseManager/isPassing", isPassing);
  }

  public void setPassingPoseIndex(int index) {
    indexOfPassingPose = index;
  }

  // operations
  public void iteratePassingPose(boolean forwards) {

    if (forwards) {
      indexOfPassingPose++;

      if (indexOfPassingPose == maxIndexOfPassingPose) {
        indexOfPassingPose = 0;
      }
    } else {
      indexOfPassingPose--;

      if (indexOfPassingPose == -1) {
        indexOfPassingPose = maxIndexOfPassingPose - 1;
      }
    }

    Logger.recordOutput("GoalPoseManager/PassingPoseName", getPassingPoseName());
    // Logger.recordOutput("GoalPoseManager/targetPose", getTargetPose());
  }

  // getters
  public String getPassingPoseName() {
    return DriveConstants.PASSING_GOALS_NAMES().get(indexOfPassingPose);
  }

  public Pose2d getTargetPose() {

    if (isPassing) {
      return DriveConstants.GET_PASSING_GOAL(indexOfPassingPose);
    } else {
      return DriveConstants.GOAL_POSE.get();
    }
  }

  // logging it here bc this hte only place we log it at
  public void setIsPassing(boolean isPassing) {
    this.isPassing = isPassing;
    Logger.recordOutput("GoalPoseManager/isPassing", isPassing);
  }

  // commands
  public Command iteratePassingPoseCommand(boolean forwards) {
    return new InstantCommand(() -> iteratePassingPose(forwards));
  }

  public Command setPassingPoseIndexCommand(int index) {
    return new InstantCommand(() -> setPassingPoseIndex(index));
  }
}
