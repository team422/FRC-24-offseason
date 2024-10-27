package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim m_topFlywheel;
  private FlywheelSim m_bottomFlywheel;
  private double m_topVoltage;
  private double m_bottomVoltage;

  public FlywheelIOSim() {
    m_topFlywheel =
        new FlywheelSim(
            ShooterConstants.kSimTopGearbox,
            ShooterConstants.kSimTopGearing,
            ShooterConstants.kSimTopMOI);
    m_bottomFlywheel =
        new FlywheelSim(
            ShooterConstants.kSimBottomGearbox,
            ShooterConstants.kSimBottomGearing,
            ShooterConstants.kSimBottomMOI);
  }

  @Override
  public void setVoltage(double topVoltage, double bottomVoltage) {
    topVoltage = MathUtil.clamp(topVoltage, -12, 12);
    bottomVoltage = MathUtil.clamp(bottomVoltage, -12, 12);

    m_topFlywheel.setInputVoltage(topVoltage);
    m_bottomFlywheel.setInputVoltage(bottomVoltage);

    m_topVoltage = topVoltage;
    m_bottomVoltage = bottomVoltage;
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    m_topFlywheel.update(0.02);
    m_bottomFlywheel.update(0.02);

    inputs.topVelocityRPS = m_topFlywheel.getAngularVelocityRPM() / 60;
    inputs.topVoltage = m_topVoltage;
    inputs.topCurrentAmps = m_topFlywheel.getCurrentDrawAmps();
    inputs.bottomVelocityRPS = m_bottomFlywheel.getAngularVelocityRPM() / 60;
    inputs.bottomVoltage = m_bottomVoltage;
    inputs.bottomCurrentAmps = m_bottomFlywheel.getCurrentDrawAmps();
  }
}
