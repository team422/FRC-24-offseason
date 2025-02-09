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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Ports;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {

  private final CANSparkMax m_driveSparkMax;
  private final CANSparkMax m_turnSparkMax;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;
  private final CANcoder m_cancoder;
  private final Queue<Double> m_timestampQueue;
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_turnPositionQueue;

  private StatusSignal<Double> m_turnAbsolutePosition;

  private final boolean m_isTurnMotorInverted = true;
  private final Rotation2d m_absoluteEncoderOffset;

  // for calculating acceleration
  private double m_lastVelocity = 0.0;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        m_driveSparkMax = new CANSparkMax(Ports.kFrontLeftDrive, MotorType.kBrushless);
        m_turnSparkMax = new CANSparkMax(Ports.kFrontLeftTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kFrontLeftCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        m_driveSparkMax = new CANSparkMax(Ports.kFrontRightDrive, MotorType.kBrushless);
        m_turnSparkMax = new CANSparkMax(Ports.kFrontRightTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kFrontRightCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        m_driveSparkMax = new CANSparkMax(Ports.kBackLeftDrive, MotorType.kBrushless);
        m_turnSparkMax = new CANSparkMax(Ports.kBackLeftTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kBackLeftCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        m_driveSparkMax = new CANSparkMax(Ports.kBackRightDrive, MotorType.kBrushless);
        m_turnSparkMax = new CANSparkMax(Ports.kBackRightTurn, MotorType.kBrushless);
        m_cancoder = new CANcoder(Ports.kBackRightCancoder);
        m_absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    m_driveSparkMax.restoreFactoryDefaults();
    m_turnSparkMax.restoreFactoryDefaults();

    m_driveSparkMax.setCANTimeout(250);
    m_turnSparkMax.setCANTimeout(250);

    m_driveEncoder = m_driveSparkMax.getEncoder();
    m_turnRelativeEncoder = m_turnSparkMax.getEncoder();

    m_turnSparkMax.setInverted(m_isTurnMotorInverted);
    m_driveSparkMax.setSmartCurrentLimit(40);
    m_turnSparkMax.setSmartCurrentLimit(30);
    m_driveSparkMax.enableVoltageCompensation(12.0);
    m_turnSparkMax.enableVoltageCompensation(12.0);

    m_driveEncoder.setPosition(0.0);
    m_driveEncoder.setMeasurementPeriod(10);
    m_driveEncoder.setAverageDepth(2);

    m_turnRelativeEncoder.setPosition(0.0);
    m_turnRelativeEncoder.setMeasurementPeriod(10);
    m_turnRelativeEncoder.setAverageDepth(2);

    m_driveSparkMax.setCANTimeout(0);
    m_turnSparkMax.setCANTimeout(0);

    m_driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / DriveConstants.kOdometryFrequency));
    m_turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / DriveConstants.kOdometryFrequency));
    m_timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    m_drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = m_driveEncoder.getPosition();
                  if (m_driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    m_turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = m_turnRelativeEncoder.getPosition();
                  if (m_turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    m_driveSparkMax.burnFlash();
    m_turnSparkMax.burnFlash();

    CANcoderConfiguration config = new CANcoderConfiguration();
    m_cancoder.getConfigurator().refresh(config.MagnetSensor);
    m_cancoder.getConfigurator().apply(config);

    m_turnAbsolutePosition = m_cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.kOdometryFrequency, m_turnAbsolutePosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(m_turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(m_driveEncoder.getPosition()) / DriveConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_driveEncoder.getVelocity())
            / DriveConstants.kDriveGearRatio;
    inputs.driveAccelerationRadPerSecSq = calculateDriveAcceleration(0.02);
    inputs.driveAppliedVolts = m_driveSparkMax.getAppliedOutput() * m_driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {m_driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble())
            .minus(m_absoluteEncoderOffset);

    // For SparkMax, the relative encoders can't be trusted so we will always use the cancoder
    inputs.turnPosition = inputs.turnAbsolutePosition;
    // inputs.turnPosition =
    //     Rotation2d.fromRotations(
    //         m_turnRelativeEncoder.getPosition() / DriveConstants.kTurnGearRatio);

    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_turnRelativeEncoder.getVelocity())
            / DriveConstants.kTurnGearRatio;
    inputs.turnAppliedVolts = m_turnSparkMax.getAppliedOutput() * m_turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {m_turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value) / DriveConstants.kDriveGearRatio)
            .toArray();
    inputs.odometryTurnPositions =
        m_turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.kTurnGearRatio))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    m_driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    m_turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  private double calculateDriveAcceleration(double dt) {
    double velocity = m_driveEncoder.getVelocity();
    double acceleration = (velocity - m_lastVelocity) / dt;
    m_lastVelocity = velocity;
    return acceleration;
  }
}
