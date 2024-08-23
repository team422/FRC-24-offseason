package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO m_io;
  public final IntakeInputsAutoLogged m_inputs;
  private SubsystemProfiles m_profiles;

  public enum IntakeState {
    kIdle,
    kIntaking,
    kOuttaking
  }

  public Intake(IntakeIO io) {
    m_io = io;
    m_inputs = new IntakeInputsAutoLogged();

    m_profiles = new SubsystemProfiles(IntakeState.class, new HashMap<>(), IntakeState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    Logger.processInputs("Intake", m_inputs);
    Logger.recordOutput("Intake/CurrentState", (IntakeState) m_profiles.getCurrentProfile());
  }

  public void updateState(IntakeState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(IntakeConstants.kIdleVoltage);
        break;
      case kIntaking:
        m_io.setVoltage(IntakeConstants.kIntakeVoltage);
        break;
      case kOuttaking:
        m_io.setVoltage(IntakeConstants.kOuttakeVoltage);
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  //Written by Ronith Kollipara
  //priyanka
  //olivia
  //Completed intajes and kicker today vs
}
