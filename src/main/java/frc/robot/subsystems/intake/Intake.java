package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
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
    kVomitting
  }

  public Intake(IntakeIO io) {
    m_io = io;
    m_inputs = new IntakeInputsAutoLogged();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IntakeState.kIdle, this::idlePeriodic);
    periodicHash.put(IntakeState.kIntaking, this::intakingPeriodic);
    periodicHash.put(IntakeState.kVomitting, this::vomittingPeriodic);
    m_profiles = new SubsystemProfiles(IntakeState.class, periodicHash, IntakeState.kIdle);
  }

  @Override
  public void periodic() {
    var start = Timer.getFPGATimestamp();

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Intake", m_inputs);
    Logger.recordOutput("Intake/CurrentState", (IntakeState) m_profiles.getCurrentProfile());

    Logger.recordOutput("PeriodicTime/Intake", Timer.getFPGATimestamp() - start);
  }

  private void idlePeriodic() {
    m_io.setVoltage(IntakeConstants.kIdleVoltage.get());
  }

  private void intakingPeriodic() {
    m_io.setVoltage(IntakeConstants.kIntakeVoltage.get());
  }

  private void vomittingPeriodic() {
    m_io.setVoltage(IntakeConstants.kOuttakeVoltage.get());
  }

  public void updateState(IntakeState state) {
    switch (state) {
      case kIdle:
        m_io.setVoltage(IntakeConstants.kIdleVoltage.get());
        break;
      case kIntaking:
        m_io.setVoltage(IntakeConstants.kIntakeVoltage.get());
        break;
      case kVomitting:
        m_io.setVoltage(IntakeConstants.kOuttakeVoltage.get());
        break;
    }

    m_profiles.setCurrentProfile(state);
  }

  public IntakeState getCurrentState() {
    return (IntakeState) m_profiles.getCurrentProfile();
  }

  // Written by Ronith Kollipara
  // priyanka
  // olivia
  // Completed intajes and kicker today vs
}
