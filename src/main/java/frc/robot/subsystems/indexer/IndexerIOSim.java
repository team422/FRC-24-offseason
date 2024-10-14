package frc.robot.subsystems.indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.Ports;
import frc.robot.util.LoggedTunableNumber;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim m_sim;
  private PIDController m_controller;
  private double m_voltage;
  private boolean m_isNeutral;
  private DigitalInput m_photoElectricOne;
  private DigitalInput m_photoElectricTwo;

  public IndexerIOSim() {
    m_sim =
        new DCMotorSim(
            IndexerConstants.kSimGearbox, IndexerConstants.kSimGearing, IndexerConstants.kSimMOI);

    m_controller =
        new PIDController(
            IndexerConstants.kIndexerP.get(),
            IndexerConstants.kIndexerI.get(),
            IndexerConstants.kIndexerD.get());

    m_voltage = 0.0;
    m_isNeutral = true;

    m_photoElectricOne = new DigitalInput(Ports.kPhotoElectricOne);
    m_photoElectricTwo = new DigitalInput(Ports.kPhotoElectricTwo);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          System.out.println(
              IndexerConstants.kIndexerP.get()
                  + " "
                  + IndexerConstants.kIndexerI.get()
                  + " "
                  + IndexerConstants.kIndexerD.get());
          m_controller.setP(IndexerConstants.kIndexerP.get());
          m_controller.setI(IndexerConstants.kIndexerI.get());
          m_controller.setD(IndexerConstants.kIndexerD.get());
        },
        IndexerConstants.kIndexerP,
        IndexerConstants.kIndexerI,
        IndexerConstants.kIndexerD);

    if (m_isNeutral) {
      m_voltage = 0.0;
    } else {
      m_voltage = m_controller.calculate(m_sim.getAngularVelocityRPM() / 60);
    }

    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.angularVelocityRPS = m_sim.getAngularVelocityRPM() / 60;
    inputs.desiredVelocityRPS = m_controller.getSetpoint();
    inputs.voltage = m_voltage;

    inputs.photoElectricOne = photoElectricOneDetected();
    inputs.photoElectricTwo = photoElectricTwoDetected();
    inputs.hasGamePiece = hasGamePiece();
  }

  @Override
  public void setDesiredVelocity(double velocityRPS) {
    m_isNeutral = false;
    m_controller.setSetpoint(velocityRPS);
  }

  @Override
  public void setNeutral() {
    m_isNeutral = true;
    m_controller.setSetpoint(0.0);
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
