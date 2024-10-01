package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotAction;
import frc.robot.subsystems.kicker.Kicker.KickerState;

public class AutoShoot extends Command {
  private Timer m_timer = new Timer();

  @Override
  public void initialize() {
    RobotState.getInstance().updateRobotAction(RobotAction.kAutoShoot);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    if (m_timer.get() > 2) {
      RobotState.getInstance().setKicker(KickerState.kShooting);
      RobotState.getInstance().setDefaultAction();
      return true;
    }
    return RobotState.getInstance().getRobotAction() != RobotAction.kAutoShoot;
  }
}
