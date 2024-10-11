package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.Ports;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim m_sim;
  private double m_voltage;
  private DIOSim m_photoElectricOneSim;
  private DIOSim m_photoElectricTwoSim;

  public IndexerIOSim() {
    m_sim =
        new DCMotorSim(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimGearing, IndexerConstants.kSimMOI);

    m_voltage = 0.0;

    m_photoElectricOneSim = new DIOSim(Ports.kPhotoElectricOne);
    m_photoElectricTwoSim = new DIOSim(Ports.kPhotoElectricTwo);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.angularVelocityRPM = m_sim.getAngularVelocityRPM();
    inputs.voltage = m_voltage;

    inputs.photoElectricOne = !m_photoElectricOneSim.getValue();
    inputs.photoElectricTwo = !m_photoElectricTwoSim.getValue();
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public boolean hasGamePiece() {
    return m_photoElectricOneSim.getValue() || m_photoElectricTwoSim.getValue();
  }
}
