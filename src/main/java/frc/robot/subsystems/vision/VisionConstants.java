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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static final String ARDUCAM_LEFT_NAME = "Arducam_Left";
  public static final String ARDUCAM_RIGHT_NAME = "Arducam_Right";
  public static final String LIMELIGHT_NAME = "limelight";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d ROBOT_TO_ARDUCAM_LEFT =
      new Transform3d(
          0,
          0.3,
          0.65,
          new Rotation3d(0.0, .35, -0.175)); // unused due to no left camera on robot yet
  //  public static final Transform3d
  //   ROBOT_TO_ARDUCAM_RIGHT = // ORGNL TRANS: (0.17, -0.24, 0.76,) ROT:  (0.0, 0.3909538,
  //       // 0.0174533)
  //       new Transform3d(0.2286, -0.2002028, 0.739902, new Rotation3d(0.0, 0.174533, 0.349066));
  public static final Transform3d
      ROBOT_TO_ARDUCAM_RIGHT = // ORGNL TRANS: (0.17, -0.24, 0.76,) ROT:  (0.0, 0.3909538,
          // 0.0174533)
          // new original Y is -0.2002028
          new Transform3d(0.2794, -0.24, 0.739902, new Rotation3d(0.0, 0.174533, 0.349066));

  // object detection
  // PLACEHOLDER VALUE
  public static final Transform3d ROBOT_TO_ARDUCAM_DETECTION =
      new Transform3d(0.0381, 0.0127, 0.4318, new Rotation3d(0.0610865, -0.453786, 0));

  // PLACEHOLDER VALUE
  public static final Distance ALGAE_RADIUS = Units.Inches.of(16.25 / 2);

  public static final double objTimeoutTimeSec = 0.2;
  public static final int maxObjAmount = 20;

  // Basic filtering thresholds
  public static final double maxAmbiguity = 0.2;
  public static final double maxDistance = 0.1;
  public static final double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
