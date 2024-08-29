package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.KickerConstants;

public class KickerIOSim implements KickerIO {
  private DCMotorSim m_sim;
  private double m_voltage;

  public KickerIOSim() {
    m_sim =
        new DCMotorSim(
            KickerConstants.kSimGearbox, KickerConstants.kSimGearing, KickerConstants.kSimMOI);

    m_voltage = 0.0;
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.angularVelocityRPM = m_sim.getAngularVelocityRPM();
    inputs.voltage = m_voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }
}
