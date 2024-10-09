package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.Ports;

public class KickerIOSim implements KickerIO {
  private DCMotorSim m_sim;
  private double m_voltage;
  private DIOSim m_beamBreakOneSim;
  private DIOSim m_beamBreakTwoSim;

  public KickerIOSim() {
    m_sim =
        new DCMotorSim(
            KickerConstants.kSimGearbox, KickerConstants.kSimGearing, KickerConstants.kSimMOI);

    m_voltage = 0.0;

    m_beamBreakOneSim = new DIOSim(Ports.kBeamBreakOne);
    m_beamBreakTwoSim = new DIOSim(Ports.kBeamBreakTwo);
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.angularVelocityRPM = m_sim.getAngularVelocityRPM();
    inputs.voltage = m_voltage;

    inputs.beamBreakOne = m_beamBreakOneSim.getValue();
    inputs.beamBreakTwo = m_beamBreakTwoSim.getValue();
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public boolean hasGamePiece() {
    return m_beamBreakOneSim.getValue() || m_beamBreakTwoSim.getValue();
  }
}
