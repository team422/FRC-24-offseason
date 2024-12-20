package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveProfiles;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToPoint extends Command {
  private Drive m_drive;
  private Pose2d m_targetPose;

  private PIDController m_xController =
      new PIDController(
          DriveConstants.kDriveToPointP.get(),
          DriveConstants.kDriveToPointI.get(),
          DriveConstants.kDriveToPointD.get());
  private PIDController m_yController =
      new PIDController(
          DriveConstants.kDriveToPointP.get(),
          DriveConstants.kDriveToPointI.get(),
          DriveConstants.kDriveToPointD.get());

  public DriveToPoint(Drive drive, Pose2d targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.updateProfile(DriveProfiles.kAutoAlign);
    m_drive.setDesiredHeading(m_targetPose.getRotation());

    m_xController.setPID(
        DriveConstants.kDriveToPointP.get(),
        DriveConstants.kDriveToPointI.get(),
        DriveConstants.kDriveToPointD.get());
    m_yController.setPID(
        DriveConstants.kDriveToPointP.get(),
        DriveConstants.kDriveToPointI.get(),
        DriveConstants.kDriveToPointD.get());

    m_xController.reset();
    m_yController.reset();
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_xController.setPID(
              DriveConstants.kDriveToPointP.get(),
              DriveConstants.kDriveToPointI.get(),
              DriveConstants.kDriveToPointD.get());
          m_yController.setPID(
              DriveConstants.kDriveToPointP.get(),
              DriveConstants.kDriveToPointI.get(),
              DriveConstants.kDriveToPointD.get());
        },
        DriveConstants.kDriveToPointP,
        DriveConstants.kDriveToPointI,
        DriveConstants.kDriveToPointD);

    Translation2d currTranslation = m_drive.getPose().getTranslation();
    Translation2d targetTranslation = m_targetPose.getTranslation();

    double xSpeed = -m_xController.calculate(currTranslation.getX(), targetTranslation.getX());
    double ySpeed = -m_yController.calculate(currTranslation.getY(), targetTranslation.getY());

    // ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, 0.0);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            0.0,
            isFlipped
                ? m_drive.getRotation().plus(new Rotation2d(Math.PI))
                : m_drive.getRotation());
    // we don't need to worry about rotation because drive will overwrite it with auto align
    m_drive.setDesiredChassisSpeeds(speeds);

    Logger.recordOutput("DriveToPoint/xSpeed", xSpeed);
    Logger.recordOutput("DriveToPoint/ySpeed", ySpeed);
    Logger.recordOutput("DriveToPoint/targetPose", m_targetPose);
  }

  @Override
  public boolean isFinished() {
    return m_drive.getPose().getTranslation().getDistance(m_targetPose.getTranslation()) < 0.1
        && m_drive.headingWithinTolerance();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.updateProfile(DriveProfiles.kDefault);
    m_drive.setDesiredChassisSpeeds(new ChassisSpeeds());
  }
}
