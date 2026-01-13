package frc.robot.util.FieldMovement.pathfindingUtil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CDrivetrain;
import frc.robot.util.FieldMovement.VortechsUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class PathfinderVortechs {

  // the supplier for robot pose
  private Supplier<Pose2d> poseSupplier;

  // wether or not the pathfinding command is active
  @AutoLogOutput private boolean isActive = false;

  // the current rendition of the pathfinding command
  private Command pathfindingCommand;

  private PathConstraints defaultConstraints;

  // the target pose of the pathfinder
  private Pose2d targetPose;

  public PathfinderVortechs(PathConstraints constraints, Supplier<Pose2d> poseSupplier) {
    this.defaultConstraints = constraints;

    this.poseSupplier = poseSupplier;

    // initializes the non-initialized part of the class
    targetPose = new Pose2d();
    pathfindingCommand = new InstantCommand();
  }

  // pathfinding manager commands
  // schedules the command of the pathfinder(this is the only time the starting pose gets updated)
  // I would reccomend running the generate command function. This might break stuff
  public void start() {
    start(defaultConstraints);
  }

  // I would reccomend running the generate command function. This might break stuff
  public void start(PathConstraints constraintsOverride) {
    if (isActive) {
      System.out.println(
          "TRIED TO START A NEW PATHFINDING COMMAND WHEN THE OLD IS ACTIVE. PATHFINDER VORTECHS, START");
      return;
    }

    isActive = true;

    pathfindingCommand = generatePathfindingCommand(constraintsOverride);
    pathfindingCommand.schedule();
  }

  // I would reccomend running the generate command function. This might break stuff
  // stops the command.
  public void stop() {
    if (isActive == false) {
      System.out.println("TRIED TO STOP A PATH WHEN IT ISN'T ACTIVE. PATHFINDINER VORTECHS, STOP");
      return;
    }

    isActive = false;
    pathfindingCommand.cancel();
  }

  // I would reccomend running the generate command function. This might break stuff
  public void setPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  // helper command that does everything and stops the commadn when the drivetrain reaches position
  public Command runPathCommand(Supplier<Pose2d> targetPose) {
    return runPathCommand(targetPose, defaultConstraints);
  }

  public Command runPathCommand(Supplier<Pose2d> targetPose, PathConstraints constraintsOverride) {
    // stops old routine
    return new InstantCommand(() -> stop())
        // sets the target pose
        .andThen(new InstantCommand(() -> setPose(targetPose.get())))
        // starts the new routine
        .andThen(new InstantCommand(() -> start(constraintsOverride)))
        // ending logic
        .andThen(new WaitUntilCommand(() -> isOnTarget()))
        // stops hogging drivetrain

        .andThen(new InstantCommand(() -> stop()));
  }

  // getters
  // gets the distance between the current pose and the target pose
  public double getDistance() {
    return poseSupplier
        .get()
        .getTranslation()
        .getDistance(flipPoseIfNeeded(targetPose).getTranslation());
  }

  public double getRotationDifferenceRAD() {
    return poseSupplier.get().getRotation().getRadians()
        - flipPoseIfNeeded(targetPose).getRotation().getRadians();
  }

  public boolean isOnTarget() {
    return VortechsUtil.isInTolerance(getDistance(), CDrivetrain.TOLERANCE_DIST)
        && VortechsUtil.isInTolerance(getRotationDifferenceRAD(), CDrivetrain.TOLERANCE_ROT);
  }

  // helper commands
  private Command generatePathfindingCommand(PathConstraints constraints) {
    Pose2d flippedPose = flipPoseIfNeeded(targetPose);
    return AutoBuilder.pathfindToPose(flippedPose, constraints);
  }

  private Pose2d flipPoseIfNeeded(Pose2d pose2d) {
    Pose2d flippedPose;

    if (Constants.isFlipped()) {
      flippedPose = FlippingUtil.flipFieldPose(targetPose);
    } else {
      flippedPose = targetPose;
    }

    return flippedPose;
  }
}

/*

research document for layout of this class:
 */

 /*
  get the closest one from a pipeline

 sample command to allow the robot to move mechanisms when it is in position and pathfind to closest pose:

     List<Pose2d> pathPoses = new ArrayList<>();
     pathPoses.add(new Pose2d());
     pathPoses.add(new Pose2d(10, 2, new Rotation2d()));
     VortechsClosestPoseSupplier poseSupplier = new VortechsClosestPoseSupplier(pathPoses, () -> drive.getPose());


     Command command =
         pathfinderVortechs
             .runPath(() -> poseSupplier.getClosestPose())
             .alongWith(
                 new WaitUntilCommand(() -> VortechsUtil.hasReachedDistance(0.2, pathfinderVortechs))
                     .andThen(arm.setTargetHeightCommandConsistentEnd(5))
                     .andThen(elevator.setTargetHeightCommand(5)));





   */
