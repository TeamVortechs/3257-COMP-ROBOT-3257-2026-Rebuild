package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class GoalPoseManager {
  private int indexOfPassingPose;
  private int maxIndexOfPassingPose = DriveConstants.PASSING_GOALS().size();

  // wether or not hte target pose is at the shooter or at a passing area
  private boolean isPassing = true;

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
  }

  public void setPassing(boolean isPassing) {
    this.isPassing = isPassing;
  }

  public void log() {
    Logger.recordOutput("isPassing", isPassing);
    Logger.recordOutput("PassingPoseName", getPassingPoseName());
    Logger.recordOutput("targetPose", getTargetPose());
  }

  // getters
  public String getPassingPoseName() {
    return DriveConstants.PASSING_GOALS_NAMES().get(indexOfPassingPose);
  }

  public Pose2d getTargetPose() {

    if (isPassing) {
      return DriveConstants.PASSING_GOALS().get(indexOfPassingPose);
    } else {
      return DriveConstants.GOAL_POSE.get();
    }
  }

  // commands
  public Command iteratePassingPoseCommand(boolean forwards) {
    return new InstantCommand(() -> iteratePassingPose(forwards));
  }
}
