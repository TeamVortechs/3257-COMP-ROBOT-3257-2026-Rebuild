package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class GoalPoseManager {
  private int indexOfPassingPose;
  private int maxIndexOfPassingPose = DriveConstants.PASSING_GOALS().size();

  @AutoLogOutput private boolean isPassing = false;

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
    Logger.recordOutput("GoalPoseManager/targetPose", getTargetPose());
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

  public void setIsPassing(boolean isPassing) {
    this.isPassing = isPassing;
  }

  // commands
  public Command iteratePassingPoseCommand(boolean forwards) {
    return new InstantCommand(() -> iteratePassingPose(forwards));
  }
}
