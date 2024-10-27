package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class SubsystemProfiles {
  public Object[] profileInstances;
  public Enum<?> currentProfile;
  public HashMap<Enum<?>, Runnable> profilePeriodicFunctions;
  public Enum<?> lastProfile;
  public Class<?> ProfileEnum;

  public SubsystemProfiles(
      Class<? extends Enum<?>> ProfileEnum,
      HashMap<Enum<?>, Runnable> profilePeriodicFunctions,
      Enum<?> defaultProfile) {
    this.ProfileEnum = ProfileEnum;
    profileInstances = ProfileEnum.getEnumConstants();
    currentProfile = defaultProfile;
    this.profilePeriodicFunctions = profilePeriodicFunctions;
    lastProfile = currentProfile;
  }

  public Runnable getPeriodicFunction() {
    return profilePeriodicFunctions.getOrDefault(
        currentProfile,
        () -> {
          System.out.println(
              String.format(
                  "WARNING: No periodic function for profile %s::%s",
                  ProfileEnum.getSimpleName(), currentProfile.toString()));
        });
  }

  public void setCurrentProfile(Enum<?> profile) {
    lastProfile = currentProfile;
    currentProfile = profile;
    Logger.recordOutput(
        String.format("%s/Set", ProfileEnum.getSimpleName()),
        String.format(
            "%s to %s: %f",
            lastProfile.toString(), currentProfile.toString(), Timer.getFPGATimestamp()));
  }

  public Enum<?> getCurrentProfile() {
    return currentProfile;
  }

  public void revertToLastProfile() {
    setCurrentProfile(lastProfile);
  }
}
