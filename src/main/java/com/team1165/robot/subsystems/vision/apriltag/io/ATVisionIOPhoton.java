/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2024 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */
package com.team1165.robot.subsystems.vision.apriltag.io;

import com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * {@link ATVisionIO} class that implements a AprilTag pose estimation camera powered by a
 * coprocessor running PhotonVision.
 */
public class ATVisionIOPhoton implements ATVisionIO {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;

  /**
   * Creates a new {@link ATVisionIOPhoton} with the specified constants.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public ATVisionIOPhoton(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  /**
   * Updates a {@link ATVisionIOInputs} instance with the latest updates from this {@link
   * ATVisionIO}.
   *
   * @param inputs A {@link ATVisionIOInputs} instance to update.
   */
  @Override
  public void updateInputs(ATVisionIOInputs inputs) {
    // Log if the camera is connected
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Add pose observation
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var tag : result.targets) {
          totalTagDistance += tag.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size())); // Average tag distance
      } else if (!result.targets.isEmpty()) { // Check and see if there are any targets at all
        PhotonTrackedTarget tag = result.targets.get(0);
        Optional<Pose3d> tagPose = ATVisionConstants.aprilTagLayout.getTagPose(tag.fiducialId);
        if (tagPose.isPresent()) { // If there is a tag, run single tag calculation
          tagIds.add((short) tag.fiducialId);
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(),
                  tagPose
                      .get()
                      .transformBy(tag.bestCameraToTarget.inverse())
                      .transformBy(robotToCamera.inverse()),
                  result.targets.get(0).poseAmbiguity,
                  1,
                  tag.bestCameraToTarget.getTranslation().getNorm()));
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
