package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;

public class GoalPoseManager {
    private int indexOfPassingPose;
    private int maxIndexOfPassingPose = DriveConstants.PASSING_GOALS().size();

    public void iteratePassingPose() {
        indexOfPassingPose++;

        if(indexOfPassingPose == maxIndexOfPassingPose) {
            indexOfPassingPose = 0;
        }
    }
}
