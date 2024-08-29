package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private KickerIO m_io;
  public final KickerInputsAutoLogged m_inputs;
  private SubsystemProfiles m_profiles;

  public enum KickerState {
    kIdle,
    kShooting,
    kEjecting
  }

  public Kicker(KickerIO io) {
    m_io = io;
    m_inputs = new KickerInputsAutoLogged();

    m_profiles = new SubsystemProfiles(KickerState.class, new HashMap<>(), KickerState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    Logger.processInputs("Kicker", m_inputs);
    Logger.recordOutput("Kicker/State", (KickerState) m_profiles.getCurrentProfile());
  }

  public void updateState(KickerState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(KickerConstants.kIdleVoltage.get());
        break;
      case kShooting:
        m_io.setVoltage(KickerConstants.kShootingVoltage.get());
        break;
      case kEjecting:
        m_io.setVoltage(KickerConstants.kEjectingVoltage.get());
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  // Nikki P in the skibidi house!
  // Mr. Wood better than u - Aahil
  // I love God and Jesus - Patty lin
}
