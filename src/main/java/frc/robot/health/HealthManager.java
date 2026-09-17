package frc.robot.health;

import com.team233.config.BlinkingBooleanBox;
import com.team233.mechanisms.vision.CameraHealth;
import com.team233.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.subsystems.Vision.limelight.Limelight;
import frc.robot.util.scheduling.SubsystemPriority;

public class HealthManager extends StateMachineSubsystem<HealthState> {
  private final Limelight frontLimelight;
  private final Limelight backLimelight;

  private boolean localizationHealthy = true;
  private boolean allCamerasHealthy = true;

  private final BlinkingBooleanBox localizationBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/LocalizationHealthyBox", false, true);
  private final BlinkingBooleanBox allCamerasBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/AllCamerasHealthyBox", false, true);

  public HealthManager(
      Limelight frontLimelight,
      Limelight backLimelight) {
    super(SubsystemPriority.HEALTH, HealthState.DEFAULT_STATE);

    this.frontLimelight = frontLimelight;
    this.backLimelight = backLimelight;
  }

  public boolean isAllCamerasHealthy() {
    return allCamerasHealthy;
  }


  public boolean isLocalizationHealthy() {
    return localizationHealthy
        && DSOptions.USE_TAG_LIMELIGHTS.getAsBoolean()
        && !DSOptions.PIT_FUNCTIONALITY.getAsBoolean();
  }

  @Override
  protected void collectInputs() {
    localizationHealthy =
        RobotBase.isSimulation()
            || frontLimelight.getCameraHealth() != CameraHealth.OFFLINE
            || backLimelight.getCameraHealth() != CameraHealth.OFFLINE;
    allCamerasHealthy =
        RobotBase.isSimulation()
            || (frontLimelight.getCameraHealth() != CameraHealth.OFFLINE
                && backLimelight.getCameraHealth() != CameraHealth.OFFLINE);
  }

  @Override
  protected void whileInState(HealthState state) {
    DogLog.log("Health/LocalizationHealthy", localizationHealthy);
    DogLog.log("Health/AllCamerasHealthy", allCamerasHealthy);

    localizationBlinkingBooleanBox.update(localizationHealthy);
    allCamerasBlinkingBooleanBox.update(allCamerasHealthy);
  }
}
