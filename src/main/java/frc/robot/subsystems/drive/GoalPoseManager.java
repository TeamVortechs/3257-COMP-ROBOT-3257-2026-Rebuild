package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class GoalPoseManager {
  private int indexOfPassingPose;
  private int maxIndexOfPassingPose = DriveConstants.PASSING_GOALS().size();

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

  public void periodic() {
    Logger.recordOutput("GoalPoseManager/PassingPoseName", getPassingPoseName());
    Logger.recordOutput("GoalPoseManager/targetPoseShooting", getTargetPose(false));
    Logger.recordOutput("GoalPoseManager/targetPosePassing", getTargetPose(true));
  }

  // getters
  public String getPassingPoseName() {
    return DriveConstants.PASSING_GOALS_NAMES().get(indexOfPassingPose);
  }

  public Pose2d getTargetPose(boolean isPassing) {

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
