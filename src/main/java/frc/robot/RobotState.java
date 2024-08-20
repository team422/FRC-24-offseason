package frc.robot;

import frc.robot.subsystems.drive.Drive;

@SuppressWarnings("unused")
public class RobotState {
  private static RobotState m_instance;

  // Subsystems
  private Drive m_drive;

  private RobotState(Drive drive) {
    m_drive = drive;
  }

  public static void start(Drive drive) {
    m_instance = new RobotState(drive);
  }

  public static RobotState getInstance() {
    if (m_instance == null) {
      throw new IllegalStateException("RobotState must be initialized before it can be retrieved");
    }
    return m_instance;
  }

  public void updateRobotState() {}
}
