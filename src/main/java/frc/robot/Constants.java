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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    public static final double kTrackWidthX = Units.inchesToMeters(21.5);
    public static final double kTrackWidthY = Units.inchesToMeters(20.5);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;

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

    public static final double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTurnGearRatio = 150.0 / 7.0;
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
    ;

    // TODO: Update these values
    public static final Pose3d[] kCameraPoses =
        new Pose3d[] {
          new Pose3d(new Translation3d(), new Rotation3d()),
          new Pose3d(new Translation3d(), new Rotation3d())
        };
  }

  public static final class IntakeConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("IntakeIdleVoltage", 0.0);
    public static final LoggedTunableNumber kIntakeVoltage =
        new LoggedTunableNumber("IntakeVoltage", 7.0);
    public static final LoggedTunableNumber kOuttakeVoltage =
        new LoggedTunableNumber("OuttakeVoltage", -7.0);

    // Simulation constants
    public static final DCMotor kSimGearbox = DCMotor.getNEO(1);
    public static final double kSimGearing = 1.0;
    public static final double kSimRadius = Units.inchesToMeters(3);
    public static final double kSimMass = 1.0; // in kg
    public static final double kSimMOI = 0.5 * kSimMass * kSimRadius * kSimRadius;
  }

  public static final class KickerConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("KickerIdleVoltage", 0.0);
    public static final LoggedTunableNumber kShootingVoltage =
        new LoggedTunableNumber("KickerShootingVoltage", 7.0);
    public static final LoggedTunableNumber kEjectingVoltage =
        new LoggedTunableNumber("KickerEjectVoltage", -7.0);

    public static final LoggedTunableNumber kShootingTimeout =
        new LoggedTunableNumber("KickerShootingTimeout", 1.0);

    // Simulation constants
    public static final DCMotor kSimGearbox = DCMotor.getNEO(1);
    public static final double kSimGearing = 1.0;
    public static final double kSimRadius = Units.inchesToMeters(3);
    public static final double kSimMass = 0.85; // in kg
    public static final double kSimMOI = 0.5 * kSimMass * kSimRadius * kSimRadius;
  }

  public static final class ShooterConstants {
    public static final LoggedTunableNumber kIdleVoltage =
        new LoggedTunableNumber("ShooterIdleVoltage", 0.0);
    public static final LoggedTunableNumber kEjectingVoltage =
        new LoggedTunableNumber("ShooterEjectingVoltage", 7.0);
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel P", 1.0);
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel I", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel D", 0.0);

    public static final PIDController kTopController =
        new PIDController(kP.get(), kI.get(), kD.get());
    public static final PIDController kBottomController =
        new PIDController(kP.get(), kI.get(), kD.get());

    public static final LoggedTunableNumber kTopKS =
        new LoggedTunableNumber("Top Flywheel kS", 0.0);
    public static final LoggedTunableNumber kTopKV =
        new LoggedTunableNumber("Top Flywheel kV", 0.0);
    public static final LoggedTunableNumber kBottomKS =
        new LoggedTunableNumber("Bottom Flywheel kS", 0.0);
    public static final LoggedTunableNumber kBottomKV =
        new LoggedTunableNumber("Bottom Flywheel kV", 0.0);

    public static final LoggedTunableNumber kAmpTopVelocity =
        new LoggedTunableNumber("AmpTopVelocity", 5.0);
    public static final LoggedTunableNumber kAmpBottomVelocity =
        new LoggedTunableNumber("AmpBottomVelocity", 7.0);

    // Simulation constants
    public static final DCMotor kSimTopGearbox = DCMotor.getNEO(1);
    public static final DCMotor kSimBottomGearbox = DCMotor.getNEO(1);
    public static final double kSimGearing = 0.3;
    public static final double kSimRadius = Units.inchesToMeters(4);
    public static final double kSimMass = 2 * Units.lbsToKilograms(0.23);
    public static final double kSimMOI = 0.5 * kSimMass * kSimRadius * kSimRadius;
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

    public static final int kIntakeNeo = 1;
    public static final int kKickerNeo = 2;
    public static final int kTopFlywheel = 3;
    public static final int kBottomFlywheel = 4;
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
