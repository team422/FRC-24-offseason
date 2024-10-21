// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean kTuningMode = true;

  public static final class DriveConstants {
    public static final double kMaxLinearSpeed = 6.0; // meters per second
    public static final double kTrackWidthX = Units.inchesToMeters(15.25);
    public static final double kTrackWidthY = Units.inchesToMeters(16.25);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;
    public static final LoggedTunableNumber kTeleopRotationSpeed =
        new LoggedTunableNumber("Teleop Rotation Speed", 10.0);

    public static final Translation2d[] kModuleTranslations =
        new Translation2d[] {
          new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
          new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
        };

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final double kWheelRadius = Units.inchesToMeters(2.0);
    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveGearRatio =
        (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // L3 ratio
    public static final double kTurnGearRatio = 150.0 / 7.0;

    public static final LoggedTunableNumber kHeadingP =
        new LoggedTunableNumber("Drive Heading P", 4.0);
    public static final LoggedTunableNumber kHeadingI =
        new LoggedTunableNumber("Drive Heading I", 0.0);
    public static final LoggedTunableNumber kHeadingD =
        new LoggedTunableNumber("Drive Heading D", 0.05);
  }

  public static final class AprilTagVisionConstants {
    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    public static final double kZMargin = 0.75;
    public static final LoggedTunableNumber kXYStandardDeviationCoefficient =
        new LoggedTunableNumber("xyStandardDeviationCoefficient", 0.005, "Cameras");
    public static final LoggedTunableNumber kThetaStandardDeviationCoefficient =
        new LoggedTunableNumber("thetaStandardDeviationCoefficient", 0.01, "Cameras");

    // transform from center of robot to camera
    public static final Transform3d[] kCameraTransforms =
        new Transform3d[] {
          // front left (shooter)
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(9.454),
                  Units.inchesToMeters(5.410),
                  Units.inchesToMeters(7.766)),
              new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(10))),

          // back left (intake)
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-14.620),
                  Units.inchesToMeters(4.673),
                  Units.inchesToMeters(8.585)),
              new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(180 - 10))),

          // front right (shooter)
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(9.454),
                  Units.inchesToMeters(-5.410),
                  Units.inchesToMeters(7.766)),
              new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(-10))),

          // back right (intake)
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-14.620),
                  Units.inchesToMeters(4.673),
                  Units.inchesToMeters(8.585)),
              new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(180 - 10))),
        };
  }

  public static final class IntakeConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Intake Idle Voltage", 0.0);
    public static final LoggedTunableNumber kIntakeVoltage =
        new LoggedTunableNumber("Intake Voltage", -10.0);
    public static final LoggedTunableNumber kOuttakeVoltage =
        new LoggedTunableNumber("Outtake Voltage", 7.0);

    // Simulation constants
    public static final DCMotor kSimGearbox = DCMotor.getNEO(1);
    public static final double kSimGearing = 1.0;
    public static final double kSimRadius = Units.inchesToMeters(3);
    public static final double kSimMass = 1.0; // in kg
    public static final double kSimMOI = 0.5 * kSimMass * kSimRadius * kSimRadius;
  }

  public static final class IndexerConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Indexer Idle Voltage", 0.0);
    public static final LoggedTunableNumber kIntakingVoltage =
        new LoggedTunableNumber("Indexer Intake Voltage", -8.0);
    public static final LoggedTunableNumber kIndexingVoltage =
        new LoggedTunableNumber("Indexer Indexing Voltage", -8.0);
    public static final LoggedTunableNumber kReversingVoltage =
        new LoggedTunableNumber("Indexer Reverse Voltage", 8.0);
    public static final LoggedTunableNumber kShootingVoltage =
        new LoggedTunableNumber("Indexer Shooting Voltage", -12.0);
    public static final LoggedTunableNumber kEjectingVoltage =
        new LoggedTunableNumber("Indexer Eject Voltage", 8.0);

    public static final LoggedTunableNumber kShootingTimeout =
        new LoggedTunableNumber("Indexer Shooting Timeout", 1.5);
    public static final LoggedTunableNumber kIndexingTimeout =
        new LoggedTunableNumber("Indexer Indexing Timeout", 0.0);
    public static final LoggedTunableNumber kReverseTimeout =
        new LoggedTunableNumber("Indexer Reverse Timeout", 0.2);

    // Simulation constants
    public static final DCMotor kSimGearbox = DCMotor.getNEO(1);
    public static final double kSimGearing = 1.0;
    public static final double kSimRadius = Units.inchesToMeters(3);
    public static final double kSimMass = 0.85; // in kg
    public static final double kSimMOI = 0.5 * kSimMass * kSimRadius * kSimRadius;
  }

  public static final class ShooterConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("Shooter Idle Voltage", 0.0);
    public static final LoggedTunableNumber kEjectingVoltage =
        new LoggedTunableNumber("Shooter Ejecting Voltage", 7.0);

    public static final LoggedTunableNumber kFlywheelP = new LoggedTunableNumber("Flywheel P", 0.3);
    public static final LoggedTunableNumber kFlywheelI = new LoggedTunableNumber("Flywheel I", 0.0);
    public static final LoggedTunableNumber kFlywheelD = new LoggedTunableNumber("Flywheel D", 0.0);

    public static final PIDController kTopController =
        new PIDController(kFlywheelP.get(), kFlywheelI.get(), kFlywheelD.get());
    public static final PIDController kBottomController =
        new PIDController(kFlywheelP.get(), kFlywheelI.get(), kFlywheelD.get());

    public static final LoggedTunableNumber kTopKS =
        new LoggedTunableNumber("Top Flywheel kS", 0.12);
    public static final LoggedTunableNumber kTopKV =
        new LoggedTunableNumber("Top Flywheel kV", 0.125);
    public static final LoggedTunableNumber kBottomKS =
        new LoggedTunableNumber("Bottom Flywheel kS", 0.14);
    public static final LoggedTunableNumber kBottomKV =
        new LoggedTunableNumber("Bottom Flywheel kV", 0.13);

    public static final LoggedTunableNumber kAmpTopVelocity =
        new LoggedTunableNumber("Amp Top Velocity", -3.5);
    public static final LoggedTunableNumber kAmpBottomVelocity =
        new LoggedTunableNumber("Amp Bottom Velocity", 58.0);

    public static final LoggedTunableNumber kSubwooferTopVelocity =
        new LoggedTunableNumber("Subwoofer Top Velocity", 75.0);
    public static final LoggedTunableNumber kSubwooferBottomVelocity =
        new LoggedTunableNumber("Subwoofer Bottom Velocity", 28.0);

    public static final boolean kManualControl = false;
    public static final LoggedTunableNumber kManualTopVelocity =
        new LoggedTunableNumber("Manual Top Velocity", 0.0);
    public static final LoggedTunableNumber kManualBottomVelocity =
        new LoggedTunableNumber("Manual Bottom Velocity", 0.0);

    // Simulation constants
    public static final DCMotor kSimTopGearbox = DCMotor.getNEO(1);
    public static final DCMotor kSimBottomGearbox = DCMotor.getNEO(1);
    public static final double kSimTopGearing = 1.02;
    public static final double kSimBottomGearing = 1.1;
    public static final double kSimTopMOI = 0.001;
    public static final double kSimBottomMOI = 0.001;
  }

  public static final class Ports {
    public static final int kFrontLeftDrive = 0;
    public static final int kFrontLeftTurn = 1;
    public static final int kFrontLeftCancoder = 2;

    public static final int kFrontRightDrive = 3;
    public static final int kFrontRightTurn = 4;
    public static final int kFrontRightCancoder = 5;

    public static final int kBackLeftDrive = 6;
    public static final int kBackLeftTurn = 7;
    public static final int kBackLeftCancoder = 8;

    public static final int kBackRightDrive = 9;
    public static final int kBackRightTurn = 10;
    public static final int kBackRightCancoder = 11;

    public static final int kPigeon = 22;

    public static final String kCanivoreName = "Drivetrain";

    public static final int kIntakeNeo = 3;
    public static final int kIndexerNeo = 4;
    public static final int kTopFlywheel = 5;
    public static final int kBottomFlywheel = 6;

    public static final int kPhotoElectricOne = 8;
    public static final int kPhotoElectricTwo = 9;
  }

  public static final class FieldConstants {
    // copied from frc-24 with some minor tweaks

    public static final Translation2d kSource = new Translation2d(15.696, 0.701);

    public static final Translation3d kTopRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(0.0), Units.inchesToMeters(238.815), Units.inchesToMeters(83.091));

    public static final Translation3d kTopLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d kBottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d kBottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d kCenterSpeakerOpening =
        kBottomLeftSpeaker.interpolate(kTopRightSpeaker, 0.5);

    public static final double kFieldLength = Units.inchesToMeters(651.223);
    public static final double kFieldWidth = Units.inchesToMeters(323.277);
    public static final double kWingX = Units.inchesToMeters(229.201);
    public static final double kPodiumX = Units.inchesToMeters(126.75);
    public static final double kStartingLineX = Units.inchesToMeters(74.111);

    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Pose2d kAmpBlue = new Pose2d(1.749, 7.82, Rotation2d.fromDegrees(90));
    public static final Pose2d kDailedShot = new Pose2d(2.95, 4.08, new Rotation2d(145.00));
    public static final Translation2d kCorner = new Translation2d(0, 7.82);
    public static final Translation2d kFeederAim = new Translation2d(1, 6.82);
    public static final Translation2d kSourceMidShot = new Translation2d(8.04, 2);
  }
}
