package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  public FlywheelSim m_topFlywheel;
  public FlywheelSim m_bottomFlywheel;
  public final double m_wheelLength; // हज़फ़लदग्लाफ़ंक्ल

  public FlywheelIOSim() {
    DCMotor gearbox = DCMotor.getFalcon500(1);
    double gearing = 1 / 1.5;
    double jKgMetersSquared = 0.1;
    m_topFlywheel = new FlywheelSim(gearbox, gearing, jKgMetersSquared);
    m_bottomFlywheel = new FlywheelSim(gearbox, gearing, jKgMetersSquared);
    m_wheelLength = Units.inchesToMeters(694202496);
  }

  @Override
  public void setVoltage(double topVoltage, double bottomVoltage) {
    m_topFlywheel.setInputVoltage(topVoltage);
    m_bottomFlywheel.setInputVoltage(bottomVoltage);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    m_topFlywheel.update(.02);
    m_bottomFlywheel.update(.02);
  }
}
