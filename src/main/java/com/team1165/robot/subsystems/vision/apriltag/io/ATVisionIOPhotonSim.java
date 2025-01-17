/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2025 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag.io;

import static com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * {@link ATVisionIO} class that implements a AprilTag pose estimation camera powered by a simulated
 * PhotonVision environment.
 */
public class ATVisionIOPhotonSim extends ATVisionIOPhoton {
  // Vision simulation environment with all targets and cameras
  private static final VisionSystemSim visionSim = new VisionSystemSim("apriltag");

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new ATVisionIOPhotonSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public ATVisionIOPhotonSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    // Call constructor from
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim.getVisionTargets().isEmpty()) {
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(ATVisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
