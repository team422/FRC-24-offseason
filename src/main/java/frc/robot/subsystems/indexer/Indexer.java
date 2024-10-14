package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO m_io;
  public final IndexerInputsAutoLogged m_inputs;
  private SubsystemProfiles m_profiles;

  private Timer m_shootTimeout = new Timer();
  private Timer m_indexTimeout = new Timer();
  private Timer m_reverseTimeout = new Timer();

  public enum IndexerState {
    kIdle,
    kIntaking,
    kIndexing,
    kReversing,
    kShooting,
    kVomitting
  }

  public Indexer(IndexerIO io) {
    m_io = io;
    m_inputs = new IndexerInputsAutoLogged();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IndexerState.kIdle, this::idlePeriodic);
    periodicHash.put(IndexerState.kIntaking, this::intakingPeriodic);
    periodicHash.put(IndexerState.kIndexing, this::indexingPeriodic);
    periodicHash.put(IndexerState.kReversing, this::reversingPeriodic);
    periodicHash.put(IndexerState.kShooting, this::shootingPeriodic);
    periodicHash.put(IndexerState.kVomitting, this::vomittingPeriodic);
    m_profiles = new SubsystemProfiles(IndexerState.class, periodicHash, IndexerState.kIdle);
  }

  @Override
  public void periodic() {
    double start = Timer.getFPGATimestamp();

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Indexer", m_inputs);
    Logger.recordOutput("Indexer/State", (IndexerState) m_profiles.getCurrentProfile());

    Logger.recordOutput("PeriodicTime/Indexer", Timer.getFPGATimestamp() - start);
  }

  private void idlePeriodic() {
    m_io.setVoltage(IndexerConstants.kIdleVoltage.get());
  }

  private void intakingPeriodic() {
    if (m_io.hasGamePiece()) {
      updateState(IndexerState.kIndexing);
      indexingPeriodic();
      return;
    }

    m_io.setVoltage(IndexerConstants.kIntakingVoltage.get());
  }

  private void indexingPeriodic() {
    if (m_indexTimeout.hasElapsed(IndexerConstants.kIndexingTimeout.get())) {
      updateState(IndexerState.kReversing);
      reversingPeriodic();
      return;
    }

    m_io.setVoltage(IndexerConstants.kIndexingVoltage.get());
  }

  private void reversingPeriodic() {
    if (m_reverseTimeout.hasElapsed(IndexerConstants.kReverseTimeout.get())) {
      updateState(IndexerState.kIdle);
      idlePeriodic();
      return;
    }

    m_io.setVoltage(IndexerConstants.kReversingVoltage.get());
  }

  private void shootingPeriodic() {
    if (!m_io.hasGamePiece()) {
      if (m_shootTimeout.hasElapsed(IndexerConstants.kShootingTimeout.get())) {
        updateState(IndexerState.kIdle);
        idlePeriodic();
        return;
      } else {
        m_shootTimeout.start();
      }
    }

    m_io.setVoltage(IndexerConstants.kShootingVoltage.get());
  }

  private void vomittingPeriodic() {
    m_io.setVoltage(IndexerConstants.kEjectingVoltage.get());
  }

  public void updateState(IndexerState state) {
    switch (state) {
      case kIdle:
        break;
      case kIntaking:
        break;
      case kIndexing:
        m_indexTimeout.restart();
        break;
      case kReversing:
        m_reverseTimeout.restart();
        break;
      case kShooting:
        m_shootTimeout.reset();
        break;
      case kVomitting:
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  public IndexerState getState() {
    return (IndexerState) m_profiles.getCurrentProfile();
  }

  // Nikki P in the skibidi house!
  // Mr. Wood better than u - Aahil
  // I love God and Jesus - Patty lin
  // James and Tommy better than you guys cause you're stinky and rotund like a clock
  // Ya'll need to get a life :) - also add a v8 with a turbo and a supercharger
}
