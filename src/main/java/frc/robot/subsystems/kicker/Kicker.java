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

  public enum KickerState {
    kIdle,
    kIntaking,
    kShooting,
    kVomitting
  }

  public Kicker(KickerIO io) {
    m_io = io;
    m_inputs = new KickerInputsAutoLogged();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(KickerState.kIdle, () -> {});
    periodicHash.put(KickerState.kIntaking, this::intakingPeriodic);
    periodicHash.put(KickerState.kShooting, this::shootingPeriodic);
    periodicHash.put(KickerState.kVomitting, () -> {});
    m_profiles = new SubsystemProfiles(KickerState.class, periodicHash, KickerState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Kicker", m_inputs);
    Logger.recordOutput("Kicker/State", (KickerState) m_profiles.getCurrentProfile());
  }

  private void intakingPeriodic() {
    if (m_io.hasGamePiece()) {
      updateState(KickerState.kIdle);
    }
  }

  private void shootingPeriodic() {
    if (m_shootTimeout.hasElapsed(KickerConstants.kShootingTimeout.get())) {
      updateState(KickerState.kIdle);
    }
  }

  public void updateState(KickerState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(KickerConstants.kIdleVoltage.get());
        break;
      case kIntaking:
        m_io.setVoltage(KickerConstants.kIntakingVoltage.get());
        break;
      case kShooting:
        m_io.setVoltage(KickerConstants.kShootingVoltage.get());
        m_shootTimeout.restart();
        break;
      case kVomitting:
        m_io.setVoltage(KickerConstants.kEjectingVoltage.get());
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  // Nikki P in the skibidi house!
  // Mr. Wood better than u - Aahil
  // I love God and Jesus - Patty lin
  // James and Tommy better than you guys cause you're stinky and rotund like a clock
}
