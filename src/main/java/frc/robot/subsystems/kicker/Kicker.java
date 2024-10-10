package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private KickerIO m_io;
  public final KickerInputsAutoLogged m_inputs;
  private SubsystemProfiles m_profiles;

  private Timer m_shootTimeout = new Timer();
  private Timer m_intakeTimeout = new Timer();
  private Timer m_indexTimeout = new Timer();
  private Timer m_reverseTimeout = new Timer();

  public enum KickerState {
    kIdle,
    kIntaking,
    kIndexing,
    kReversing,
    kShooting,
    kVomitting
  }

  public Kicker(KickerIO io) {
    m_io = io;
    m_inputs = new KickerInputsAutoLogged();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(KickerState.kIdle, this::idlePeriodic);
    periodicHash.put(KickerState.kIntaking, this::intakingPeriodic);
    periodicHash.put(KickerState.kIndexing, this::indexingPeriodic);
    periodicHash.put(KickerState.kReversing, this::reversingPeriodic);
    periodicHash.put(KickerState.kShooting, this::shootingPeriodic);
    periodicHash.put(KickerState.kVomitting, this::vomittingPeriodic);
    m_profiles = new SubsystemProfiles(KickerState.class, periodicHash, KickerState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Kicker", m_inputs);
    Logger.recordOutput("Kicker/State", (KickerState) m_profiles.getCurrentProfile());
  }

  private void idlePeriodic() {
    m_io.setVoltage(KickerConstants.kIdleVoltage.get());
  }

  private void intakingPeriodic() {
    m_io.setVoltage(KickerConstants.kIntakingVoltage.get());

    if (m_io.hasGamePiece() || m_intakeTimeout.hasElapsed(KickerConstants.kIntakeTimeout.get())) {
      updateState(KickerState.kIdle);
      idlePeriodic();
    }
  }

  private void indexingPeriodic() {
    m_io.setVoltage(KickerConstants.kIndexingVoltage.get());

    if (m_io.hasGamePiece() || m_indexTimeout.hasElapsed(KickerConstants.kIndexingVoltage.get())) {
      updateState(KickerState.kReversing);
      reversingPeriodic();
    }
  }

  private void reversingPeriodic() {
    m_io.setVoltage(KickerConstants.kReversingVoltage.get());

    if (m_reverseTimeout.hasElapsed(KickerConstants.kReverseTimeout.get())) {
      updateState(KickerState.kIdle);
      idlePeriodic();
    }
  }

  private void shootingPeriodic() {
    m_io.setVoltage(KickerConstants.kShootingVoltage.get());

    if (m_shootTimeout.hasElapsed(KickerConstants.kShootingTimeout.get())) {
      updateState(KickerState.kIdle);
      m_io.setVoltage(KickerConstants.kIdleVoltage.get());
    }
  }

  private void vomittingPeriodic() {
    m_io.setVoltage(KickerConstants.kEjectingVoltage.get());
  }

  public void updateState(KickerState state) {
    switch (state) {
      case kIdle:
        break;
      case kIntaking:
        m_intakeTimeout.restart();
        break;
      case kIndexing:
        m_indexTimeout.restart();
        break;
      case kReversing:
        m_reverseTimeout.restart();
        break;
      case kShooting:
        m_shootTimeout.restart();
        break;
      case kVomitting:
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  // Nikki P in the skibidi house!
  // Mr. Wood better than u - Aahil
  // I love God and Jesus - Patty lin
  // James and Tommy better than you guys cause you're stinky and rotund like a clock
  // Ya'll need to get a life :) - also add a v8 with a turbo and a supercharger
}
