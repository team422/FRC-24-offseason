package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private FlywheelIO m_io;
  public final FlywheelInputsAutoLogged m_inputs;
  private SubsystemProfiles m_profiles;
  private PIDController m_topController;
  private PIDController m_bottomController;
  private SimpleMotorFeedforward m_topFeedforward;
  private SimpleMotorFeedforward m_bottomFeedforward;

  public enum ShooterState {
    kIdle,
    kRevving,
    kEjecting,
    kAmp,
    kManualControl,
  }

  /** Class to represent a setpoint for the Shooter */
  public record ShooterPosition(double topVelocityRPS, double bottomVelocityRPS) {}

  public Shooter(
      FlywheelIO io,
      PIDController topController,
      PIDController bottomController,
      SimpleMotorFeedforward topFeedforward,
      SimpleMotorFeedforward bottomFeedforward) {
    m_io = io;
    m_topController = topController;
    m_bottomController = bottomController;
    m_topFeedforward = topFeedforward;
    m_bottomFeedforward = bottomFeedforward;

    m_inputs = new FlywheelInputsAutoLogged();

    HashMap<Enum<?>, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ShooterState.kIdle, this::idlePeriodic);
    periodicHash.put(ShooterState.kRevving, this::revvingPeriodic);
    periodicHash.put(ShooterState.kEjecting, this::ejectingPeriodic);
    periodicHash.put(ShooterState.kAmp, this::ampPeriodic);
    periodicHash.put(ShooterState.kManualControl, this::manualControlPeriodic);
    m_profiles = new SubsystemProfiles(ShooterState.class, periodicHash, ShooterState.kIdle);
  }

  @Override
  public void periodic() {
    double start = Timer.getFPGATimestamp();

    if (ShooterConstants.kManualControl) {
      updateState(ShooterState.kManualControl);
    }

    m_io.updateInputs(m_inputs);

    m_profiles.getPeriodicFunction().run();

    Logger.processInputs("Shooter", m_inputs);
    Logger.recordOutput("Shooter/State", (ShooterState) m_profiles.getCurrentProfile());

    Logger.recordOutput("PeriodicTime/Shooter", Timer.getFPGATimestamp() - start);
  }

  public void idlePeriodic() {
    m_io.setVoltage(ShooterConstants.kIdleVoltage.get(), ShooterConstants.kIdleVoltage.get());

    Logger.recordOutput("Shooter/TopSetVoltage", ShooterConstants.kIdleVoltage.get());
    Logger.recordOutput("Shooter/BottomSetVoltage", ShooterConstants.kIdleVoltage.get());
    Logger.recordOutput("Shooter/CurrentTopVelocity", m_inputs.topVelocityRPS);
    Logger.recordOutput("Shooter/CurrentBottomVelocity", m_inputs.bottomVelocityRPS);
    Logger.recordOutput("Shooter/DesiredTopVelocity", m_topController.getSetpoint());
    Logger.recordOutput("Shooter/DesiredBottomVelocity", m_bottomController.getSetpoint());
  }

  public void ejectingPeriodic() {
    m_io.setVoltage(
        ShooterConstants.kEjectingVoltage.get(), ShooterConstants.kEjectingVoltage.get());

    Logger.recordOutput("Shooter/TopSetVoltage", ShooterConstants.kEjectingVoltage.get());
    Logger.recordOutput("Shooter/BottomSetVoltage", ShooterConstants.kEjectingVoltage.get());
    Logger.recordOutput("Shooter/CurrentTopVelocity", m_inputs.topVelocityRPS);
    Logger.recordOutput("Shooter/CurrentBottomVelocity", m_inputs.bottomVelocityRPS);
    Logger.recordOutput("Shooter/DesiredTopVelocity", m_topController.getSetpoint());
    Logger.recordOutput("Shooter/DesiredBottomVelocity", m_bottomController.getSetpoint());
  }

  public void revvingPeriodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_topController.setPID(
              ShooterConstants.kFlywheelP.get(),
              ShooterConstants.kFlywheelI.get(),
              ShooterConstants.kFlywheelD.get());
          m_bottomController.setPID(
              ShooterConstants.kFlywheelP.get(),
              ShooterConstants.kFlywheelI.get(),
              ShooterConstants.kFlywheelD.get());
        },
        ShooterConstants.kFlywheelP,
        ShooterConstants.kFlywheelI,
        ShooterConstants.kFlywheelD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_topFeedforward =
              new SimpleMotorFeedforward(
                  ShooterConstants.kTopKS.get(), ShooterConstants.kTopKV.get());
        },
        ShooterConstants.kTopKS,
        ShooterConstants.kTopKV);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_bottomFeedforward =
              new SimpleMotorFeedforward(
                  ShooterConstants.kBottomKS.get(), ShooterConstants.kBottomKV.get());
        },
        ShooterConstants.kBottomKS,
        ShooterConstants.kBottomKV);

    double topSetVoltage = m_topController.calculate(m_inputs.topVelocityRPS);
    double bottomSetVoltage = m_bottomController.calculate(m_inputs.bottomVelocityRPS);

    double topFeedforwardVoltage = m_topFeedforward.calculate(m_topController.getSetpoint());
    double bottomFeedforwardVoltage =
        m_bottomFeedforward.calculate(m_bottomController.getSetpoint());

    Logger.recordOutput("Shooter/TopFeedforwardVoltage", topFeedforwardVoltage);
    Logger.recordOutput("Shooter/BottomFeedforwardVoltage", bottomFeedforwardVoltage);
    Logger.recordOutput("Shooter/TopPIDVoltage", topSetVoltage);
    Logger.recordOutput("Shooter/BottomPIDVoltage", bottomSetVoltage);

    topSetVoltage += topFeedforwardVoltage;
    bottomSetVoltage += bottomFeedforwardVoltage;

    m_io.setVoltage(topSetVoltage, bottomSetVoltage);

    Logger.recordOutput("Shooter/TopSetVoltage", topSetVoltage);
    Logger.recordOutput("Shooter/BottomSetVoltage", bottomSetVoltage);
    Logger.recordOutput("Shooter/CurrentTopVelocity", m_inputs.topVelocityRPS);
    Logger.recordOutput("Shooter/CurrentBottomVelocity", m_inputs.bottomVelocityRPS);
    Logger.recordOutput("Shooter/DesiredTopVelocity", m_topController.getSetpoint());
    Logger.recordOutput("Shooter/DesiredBottomVelocity", m_bottomController.getSetpoint());
  }

  public void ampPeriodic() {
    setDesiredVelocity(
        ShooterConstants.kAmpTopVelocity.get(), ShooterConstants.kAmpBottomVelocity.get());
    revvingPeriodic();
  }

  public void manualControlPeriodic() {
    setDesiredVelocity(
        ShooterConstants.kManualTopVelocity.get(), ShooterConstants.kManualBottomVelocity.get());
    revvingPeriodic();
  }

  public void updateState(ShooterState state) {
    switch (state) {
      case kIdle:
      case kEjecting:
        setDesiredVelocity(0, 0);
        break;

      case kAmp:
      case kRevving:
      case kManualControl:
        break;
    }
    m_profiles.setCurrentProfile(state);
  }

  public ShooterState getState() {
    return (ShooterState) m_profiles.getCurrentProfile();
  }

  public void setDesiredVelocity(double topVelocityRPS, double bottomVelocityRPS) {
    m_topController.setSetpoint(topVelocityRPS);
    m_bottomController.setSetpoint(bottomVelocityRPS);
  }

  public void setDesiredVelocity(ShooterPosition position) {
    m_topController.setSetpoint(position.topVelocityRPS());
    m_bottomController.setSetpoint(position.bottomVelocityRPS());
  }

  public ShooterPosition getVelocity() {
    return new ShooterPosition(m_inputs.topVelocityRPS, m_inputs.bottomVelocityRPS);
  }

  public void resetPID() {
    m_topController.reset();
    m_bottomController.reset();
  }
}
