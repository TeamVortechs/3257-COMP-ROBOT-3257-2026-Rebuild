// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionLoglessSim extends VisionLogless {
  VisionSystemSim visionSim;
  Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim leftSim;
  private final PhotonCameraSim rightSim;

  public VisionLoglessSim(
      VisionConsumer consumer, PowerModuleIO powerModuleIO, Supplier<Pose2d> poseSupplier) {
    super(consumer, powerModuleIO);

    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var leftProperties =
        new SimCameraProperties().setCalibration(1280, 800, Rotation2d.fromDegrees(80));
    var rightProperties =
        new SimCameraProperties().setCalibration(1280, 800, Rotation2d.fromDegrees(80));
    leftSim = new PhotonCameraSim(photonCameraLeft, leftProperties, aprilTagLayout);
    rightSim = new PhotonCameraSim(photonCameraRight, rightProperties, aprilTagLayout);
    visionSim.addCamera(leftSim, robotToPhoton0);
    visionSim.addCamera(rightSim, robotToPhoton1);
  }

  @Override
  public void periodic() {
    visionSim.update(poseSupplier.get());
    super.periodic();
  }
}
