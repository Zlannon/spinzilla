package frc.robot.power_manager;

import com.team233.mechanisms.PowerManaged;
import com.team233.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.deploy.Deploy;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.DyeRotor.DyeRotor;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PowerManager extends StateMachineSubsystem<PowerManagerState> {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();
  private final PowerManaged shooter;
  private final PowerManaged collector;
  private final PowerManaged deploy;
  private final PowerManaged dyeRotor;
  private final PowerManaged swerve;

  public PowerManager(
      Shooter shooter,
      Collector collector,
      Deploy deploy,
      DyeRotor dyeRotor,
      Swerve swerve) {
    super(SubsystemPriority.POWER_MANAGER, PowerManagerState.IDLE);
    this.shooter = shooter;
    this.collector = collector;
    this.deploy = deploy;
    this.dyeRotor = dyeRotor;
    this.swerve = swerve;
  }

  public void beastModeRequest() {
    setStateFromRequest(PowerManagerState.BEAST_MODE);
  }

  public void feedingFarRequest() {
    setStateFromRequest(PowerManagerState.FEEDING_FAR);
  }

  public void feedingRequest() {
    setStateFromRequest(PowerManagerState.FEEDING);
  }

  public void firstAutoSegmentRequest() {
    setStateFromRequest(PowerManagerState.AUTO_FIRST_SEGMENT);
  }

  public void idleRequest() {
    setStateFromRequest(PowerManagerState.IDLE);
  }

  public void prioritizeIntakeRequest() {
    setStateFromRequest(PowerManagerState.PRIORITIZE_INTAKE);
  }

  public void scoringFarRequest() {
    setStateFromRequest(PowerManagerState.SCORING_FAR);
  }

  public void scoringRequest() {
    setStateFromRequest(PowerManagerState.SCORING);
  }

  public void turboRequest() {
    setStateFromRequest(PowerManagerState.TURBO_MODE);
  }

  @Override
  protected void afterTransition(PowerManagerState newState) {
    DogLog.timestamp("PowerManager/UpdatedCurrentsAt");
    executor.execute(
        () -> {
          shooter.applyCurrentLimits(newState.shooterSupplyCurrent);
          collector.applyCurrentLimits(newState.collectorSupplyCurrent);
          deploy.applyCurrentLimits(newState.deploySupplyCurrent);
          dyeRotor.applyCurrentLimits(newState.dyeRotorSupplyCurrent);
          swerve.applyCurrentLimits(newState.swerveSupplyCurrent);
        });
  }
}
