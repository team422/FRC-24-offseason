package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;

@SuppressWarnings("unused")
public class RobotState {
  private static RobotState m_instance;

  // Subsystems
  private Drive m_drive;
  private Intake m_intake;
  private Kicker m_kicker;

  private RobotState(Drive drive, Intake intake, Kicker kicker) {
    m_drive = drive;
    m_intake = intake;
    m_kicker = kicker;
  }

  public static void start(Drive drive, Intake intake, Kicker kicker) {
    m_instance = new RobotState(drive, intake, kicker);
  }

  public static RobotState getInstance() {
    if (m_instance == null) {
      throw new IllegalStateException("RobotState must be initialized before it can be retrieved");
    }
    return m_instance;
  }

  public void updateRobotState() {}
}
