/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLog;

/**
 * A "software" interface/implementation layer for a Swerve Drivetrain powered by the CTRE Swerve
 * Library. Due to the nature of the CTRE Swerve Library, this IO class is limited in replay
 * capabilities. This IO layer is used to communicate with the CTRE Swerve Library in a partially
 * AdvantageKit compatible way. Any new features or customization that can be done outside of this
 * communication layer should be done in the actual Drive subsystem.
 */
public class DriveIO {
  /** Class used to store the IO values of a CTRE Swerve Drivetrain. */
  @AutoLog
  class DriveIOInputs extends SwerveDrivetrain.SwerveDriveState {
    /** Class used to store the IO values of a CTRE Swerve Drivetrain. */
    DriveIOInputs() {
      this.Pose = new Pose2d();
    }

    /** Make a {@link DriveIOInputs} from a {@link SwerveDrivetrain.SwerveDriveState}. */
    public void fromSwerveDriveState(SwerveDrivetrain.SwerveDriveState state) {
      this.Pose = state.Pose;
      this.Speeds = state.Speeds;
      this.ModuleStates = state.ModuleStates;
      this.ModuleTargets = state.ModuleTargets;
      this.ModulePositions = state.ModulePositions;
      this.RawHeading = state.RawHeading;
      this.Timestamp = state.Timestamp;
      this.OdometryPeriod = state.OdometryPeriod;
      this.SuccessfulDaqs = state.SuccessfulDaqs;
      this.FailedDaqs = state.FailedDaqs;
    }
  }


  /**
   * Updates a {@link DriveIOInputs} instance with the latest updates from this {@link DriveIO}.
   *
   * @param inputs A {@link DriveIOInputs} instance to update.
   */
  void updateInputs(DriveIOInputs inputs) {}

  void logModules(SwerveDrivetrain.SwerveDriveState driveState) {}

  void setControl(SwerveRequest request) {}

  void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {}

  void configNeutralMode(NeutralModeValue neutralMode) {}

  void tareEverything() {}

  void seedFeedCentric() {}

  void resetPose(Pose2d pose) {}

  void resetTranslation(Translation2d translation) {}

  void resetRotation(Rotation2d rotation) {}

  void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {}

  void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,  Matrix<N3, N1> visionMeasurementStdDevs) {}

  void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {}
}
