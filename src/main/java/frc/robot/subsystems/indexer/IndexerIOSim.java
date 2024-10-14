package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.Ports;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim m_sim;
  private double m_voltage;
  private DigitalInput m_photoElectricOne;
  private DigitalInput m_photoElectricTwo;

  public IndexerIOSim() {
    m_sim =
        new DCMotorSim(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimGearing, IndexerConstants.kSimMOI);

    m_voltage = 0.0;

    m_photoElectricOne = new DigitalInput(Ports.kPhotoElectricOne);
    m_photoElectricTwo = new DigitalInput(Ports.kPhotoElectricTwo);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.angularVelocityRPM = m_sim.getAngularVelocityRPM() / 60;

    inputs.voltage = m_voltage;

    inputs.photoElectricOne = photoElectricOneDetected();
    inputs.photoElectricTwo = photoElectricTwoDetected();
    inputs.hasGamePiece = hasGamePiece();
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public boolean hasGamePiece() {
    return photoElectricOneDetected() || photoElectricTwoDetected();
  }

  /** Returns true if the photoelectric sensor detects a piece */
  private boolean photoElectricOneDetected() {
    // in sim i still want it to be inverted so it's consistent with real
    return !m_photoElectricOne.get();
  }

  /** Returns true if the photoelectric sensor detects a piece */
  private boolean photoElectricTwoDetected() {
    // in sim i still want it to be inverted so it's consistent with real
    return !m_photoElectricTwo.get();
  }
}
