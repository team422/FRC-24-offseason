package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import org.littletonrobotics.junction.Logger;

public class AutoShootNoAlign extends Command {
  private Timer m_timer = new Timer();

  @Override
  public void initialize() {
    RobotState.getInstance().updateRobotAction(RobotAction.kAutoShootNoAlign);
    m_timer.restart();
  }

  @Override
  public boolean isFinished() {
    if (m_timer.get() > 2) {
      Logger.recordOutput("Stow/AutoShootCommand", Timer.getFPGATimestamp());
      RobotState.getInstance().setIndexer(IndexerState.kShooting);
      RobotState.getInstance().setDefaultAction();
      return true;
    }
    return RobotState.getInstance().getRobotAction() != RobotAction.kAutoShootNoAlign;
  }
}
