// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String limelightLeftName = "limelight";
  public static String photonLeftName = "Arducam_Left";
  public static String photonRightName = "Arducam_Right";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // (instead used by the vision simulation)

  // !!!! TODO: GET NEW TRANSFORMS FOR NEW ROBOT !!!!
  public static Transform3d robotToPhotonLeft =
      new Transform3d(
          -0.2921, // 12.902 in (CAD)
          0.2921, // 11.043 - 2.5 in (CAD)
          0.5334, // 0.52705 // 20 3/4 in
          new Rotation3d(
              0.0,
              -Units.degreesToRadians(25.258),
              -Units.degreesToRadians(35))); // old one is 25.3
  public static Transform3d
      robotToPhotonRight = // !! WARNING !! rough estimates. i punched this in at 9:10 PM; recheck
          // better
          new Transform3d(
              -0.2921, // 0.3277108
              -0.2921, // 0.2169922
              0.5334,
              new Rotation3d(0.0, -Units.degreesToRadians(25.258), Units.degreesToRadians(35)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2; // formerly 0.3; reduced to lower risk of Bad Things
  public static double maxZError = 0.75;
  public static int[] bannedTags = { // THIS MUST BE A SORTED LIST! I WILL BEAT YOU UP IF IT IS NOT
    1, 6, 7, 12, 17, 22, 23, 28,
  }; // all singleton tags on each TRENCH

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
