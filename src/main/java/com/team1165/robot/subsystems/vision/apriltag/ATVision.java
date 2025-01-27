/*
 * File originally made by: Mechanical Advantage - FRC 6328
 * Copyright (c) 2025 Team 6328 (https://github.com/Mechanical-Advantage)
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.vision.apriltag;

import static com.team1165.robot.subsystems.vision.apriltag.constants.ATVisionConstants.*;

import com.ctre.phoenix6.Utils;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIO;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOInputsAutoLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayDeque;
import java.util.Queue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem for a collection of cameras that estimates the pose of the robot with AprilTags
 * placed around the field.
 */
public class ATVision extends SubsystemBase {
  // IO
  private final ATVisionConsumer consumer;
  private final Alert[] disconnectedAlerts;
  private final ATVisionIO[] io;
  private final ATVisionIOInputsAutoLogged[] inputs;
  private final Transform3d[] robotToCameraTransforms = new Transform3d[1];
  private final Supplier<Rotation2d> robotRotationSupplier;

  public ATVision(
      ATVisionConsumer consumer, Supplier<Rotation2d> robotRotationSupplier, ATVisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.robotRotationSupplier = robotRotationSupplier;
    robotToCameraTransforms[0] =
        new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0.0, 0.0, 0));

    // Initialize inputs
    this.inputs = new ATVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new ATVisionIOInputsAutoLogged();
      // Update inputs once so that we can get the name value from the cameras
      io[i].updateInputs(inputs[i]);
    }

    // Initialize alerts for if a camera is disconnected
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "The AprilTag camera " + inputs[i].name + " (ID " + i + ") is disconnected.",
              AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    // Update current inputs and log inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/Camera" + i, inputs[i]);
    }

    // Initialize values to be logged from all cameras
    Queue<Pose3d> allTagPoses = new ArrayDeque<>();
    Queue<Pose3d> allRobotPoses = new ArrayDeque<>();
    Queue<Pose3d> allRobotPosesAccepted = new ArrayDeque<>();
    Queue<Pose3d> allRobotPosesRejected = new ArrayDeque<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize values to be logged from this camera
      Queue<Pose3d> tagPoses = new ArrayDeque<>();
      Queue<Pose3d> robotPoses = new ArrayDeque<>();
      Queue<Pose3d> robotPosesAccepted = new ArrayDeque<>();
      Queue<Pose3d> robotPosesRejected = new ArrayDeque<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var poseObservation : inputs[cameraIndex].poseObservations) {
        // Calculate robot pose from the camera pose provided
        Pose3d robotPose =
            poseObservation.cameraPose().plus(robotToCameraTransforms[cameraIndex].inverse());

        // Check whether to reject pose
        boolean rejectPose =
            (poseObservation.tagCount() == 1
                    && poseObservation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(robotPose.getZ()) > maxZError // Must have realistic Z coordinate
                // Must be within the field boundaries
                || robotPose.getX() < 0.0
                || robotPose.getX() > aprilTagLayout.getFieldLength()
                || robotPose.getY() < 0.0
                || robotPose.getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(robotPose);
        if (rejectPose) {
          robotPosesRejected.add(robotPose);
        } else {
          robotPosesAccepted.add(robotPose);
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(poseObservation.averageTagDistance(), 2.0) / poseObservation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            robotPose.toPose2d(),
            Utils.fpgaToCurrentTime(poseObservation.timestamp()),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Loop over single tag observations
      for (var observation : inputs[cameraIndex].singleTagObservations) {
        // Get the robot to camera transform/offset
        Transform3d robotToCamera = robotToCameraTransforms[cameraIndex];

        // Get the 2D distance from the camera to the tag
        double distance2d =
            observation.distance()
                * Math.cos(robotToCamera.getRotation().getY() + observation.ty());

        // Get the rotation of the tag from the camera position
        Rotation2d camToTagRotation =
            robotRotationSupplier
                .get()
                .plus(
                    robotToCamera
                        .getRotation()
                        .toRotation2d()
                        .plus(Rotation2d.fromRadians(observation.tx())));
        Logger.recordOutput(
            "Vision/Testing", robotToCamera.getRotation().toRotation2d().getRadians());

        // Get the 2D tag pose from the AprilTag layout
        var tagPose =
            aprilTagLayout.getTagPose(observation.tagId()).map(Pose3d::toPose2d).orElse(null);
        if (tagPose == null) {
          break;
        }

        // Get the translation from the field to the camera
        Translation2d fieldToCameraTranslation =
            new Pose2d(tagPose.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(new Transform2d(distance2d, 0, new Rotation2d()))
                .getTranslation();

        // Get the robot pose with the translation
        var cameraInverse = robotToCamera.inverse();
        var robotPose =
            new Pose2d(
                    fieldToCameraTranslation,
                    robotRotationSupplier.get().plus(robotToCamera.getRotation().toRotation2d()))
                .transformBy(
                    new Transform2d(
                        new Pose2d(
                            robotToCamera.getTranslation().toTranslation2d(),
                            robotToCamera.getRotation().toRotation2d()),
                        Pose2d.kZero));
        robotPoses.add(new Pose3d(robotPose));

        // Calculate standard deviations
        double stdDevFactor = Math.pow(observation.distance(), 2.0);
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            robotPose,
            Utils.fpgaToCurrentTime(observation.timestamp()),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera pose data
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "AprilTagVision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));

      // Add camera data to the all queues
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data from all cameras
    Logger.recordOutput("AprilTagVision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("AprilTagVision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface ATVisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
