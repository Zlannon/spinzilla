package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.team233.signals.Signals;
import com.team233.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.subsystems.DyeRotor.DyeRotor;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.Collector.CollectorState;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public final Deploy deploy;
  public final Collector collector;
  public final DyeRotor dyeRotor;
  public final CANrange hopperCANRange;
  public final DigitalInput towerSensor;

  private final Debouncer towerSensorDebouncer = new Debouncer(0.25, DebounceType.kFalling);
  private boolean towerSensorDebounced = false;

  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;
  private boolean wantsSafeStow = false;
  private boolean towerSensorRaw = false;
  private boolean ballFilling = false;

  private boolean shouldBeastMode = false;

  private final LinearFilter hopperFilter = LinearFilter.movingAverage(50);

  private double hopperDistance = 0.0;
  private double filteredDistance = 0.0;
  private double previousCanRangeDistance = 0.0;
  public static final double HIGH_CAPACITY_THRESHOLD = 6.0;
  public static final double MEDIUM_CAPACITY_THRESHOLD = 8.0;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;

  public enum HopperBallPosition {
    CLOSE_TO_SHOOTER,
    AT_SENSOR,
    BELOW_SENSOR,
  }

  private final Timer canRangeUpdateTimer = new Timer();

  private final StatusSignal<Distance> hopperDistanceSignal;

  public HopperManager(
      Deploy deploy,
      Collector collector,
      DyeRotor dyeRotor,
      CANrange hopperCANRange,
      DigitalInput towerSensor) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE_DEPLOYED);
    this.deploy = deploy;
    this.collector = collector;
    this.dyeRotor = dyeRotor;
    this.hopperCANRange = hopperCANRange;
    this.towerSensor = towerSensor;

    hopperCANRange.getConfigurator().apply(HopperManagerConfig.CAN_RANGE_CONFIG);
    canRangeUpdateTimer.start();

    hopperDistanceSignal = hopperCANRange.getDistance(false);
    Signals.forDevice(hopperCANRange).addSignals(hopperDistanceSignal);
  }

  private HopperBallPosition getShotPosition() {
    if (towerSensorDebounced) {
      return HopperBallPosition.AT_SENSOR;
    }

    if (shouldFillBalls()) {
      return HopperBallPosition.CLOSE_TO_SHOOTER;
    }

    return HopperBallPosition.BELOW_SENSOR;
  }

  public double getFeederToShooterTime() {
    return switch (getShotPosition()) {
      // TODO: Validate this
      case CLOSE_TO_SHOOTER -> 0.25;
      case AT_SENSOR -> 0.2;
      case BELOW_SENSOR -> 0.3;
    };
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    return switch (getState()) {
      case IDLE_DEPLOYED, IDLE_STOWED, INTAKING, EJECTING -> {
        yield resolveIdleState();
      }
      default -> currentState;
    };
  }

  private boolean shouldFillBalls() {
    if (towerSensorDebounced) {
      // The sensor in the tower shows we are holding fuel, so we can't fill anymore
      return false;
    }
    if (!deploy.isFullyExtended()) {
      return false;
    }
    if (!canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      // If we are using the hopper CANrange, we can start filling the tower once the hopper is
      // starting ot fill up
      return hopperCapacity == HopperCapacity.MEDIUM || hopperCapacity == HopperCapacity.HIGH;
    }

    // Otherwise, we fallback to running once we've been intaking for a few seconds
    return collector.hasBeenIntaking();
  }

  public boolean isFull() {
    if (RobotBase.isSimulation()) {
      return timeout(10.0);
    }
    return hopperCapacity == HopperCapacity.HIGH && DSOptions.USE_CANRANGE.getAsBoolean();
  }

  /** Sets conveyor and feeder to ball filling if conditions are met, otherwise idles them. */
  private void smartBallFillRequest() {
    if (shouldFillBalls()) {
      dyeRotor.ballFillingRequest();
    } else {
      dyeRotor.idleRequest();
    }
  }

  private void smartIntakeBallFillRequest() {
    if (shouldFillBalls()) {
      dyeRotor.ballFillingRequest();
    } else {
      if (towerSensorDebounced) {
        dyeRotor.idleRequest();
      } else {
        dyeRotor.intakeRequest();
      }
    }
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE_DEPLOYED -> {
        collector.idleRequest();
        deploy.intakeRequest();
        smartBallFillRequest();
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        collector.idleRequest();
        dyeRotor.idleRequest();
      }
      case IDLE_SAFE_KICKER_STOW -> {
        deploy.safeKickerStowRequest();
        collector.idleRequest();
        smartBallFillRequest();
      }
      case INTAKING -> {
        deploy.intakeRequest();
        collector.intakeRequest();
        smartIntakeBallFillRequest();
      }
      case EJECTING -> {
        deploy.intakeRequest();
        collector.ejectRequest();
        dyeRotor.ejectRequest();
      }
      case UNJAMMING -> {
        deploy.intakeRequest();
        collector.ejectRequest();
        dyeRotor.shootRequest();
      }
      case SCORE -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (deploy.getState() != DeployState.SCORE_COMPACTION
            && deploy.getState() != DeployState.SCORE_COMPACTION_WAITING) {
          deploy.intakeRequest();
        }
        collector.shootRequest();
        dyeRotor.shootRequest();
      }
      case SCORE_AND_INTAKE -> {
        deploy.intakeRequest();
        collector.intakeRequest();
        dyeRotor.shootRequest();
      }

      case FEED -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (deploy.getState() != DeployState.FEED_COMPACTION) {
          deploy.intakeRequest();
        }
        collector.shootRequest();
        dyeRotor.shootRequest();
      }
      case FEED_AND_INTAKE -> {
        deploy.intakeRequest();
        collector.intakeRequest();
        dyeRotor.shootRequest();
      }
    }
  }

  @Override
  protected void whileInState(HopperState state) {
    if (state.canBallFill) {
      if (state == HopperState.INTAKING) {
        smartIntakeBallFillRequest();
      } else {

        smartBallFillRequest();
      }
    }

    switch (state) {
      default -> {}
      case SCORE -> {
        if (shouldBeastMode) {
          deploy.beastModeRequest();
          collector.shootRequest();
          dyeRotor.shootRequest();
        } else if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          deploy.hopperCompactionRequest();
          collector.idleRequest();
          dyeRotor.shootRequest();
        } else {
          deploy.waitHopperCompactionRequest();
        }
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        collector.idleRequest();
        dyeRotor.idleRequest();
      }
      case FEED -> {
        if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          deploy.feedCompactionRequest();
          collector.idleRequest();
          dyeRotor.shootRequest();
        }
      }
    }

    if (previousCanRangeDistance != hopperDistance) {
      canRangeUpdateTimer.reset();
    }
    previousCanRangeDistance = hopperDistance;

    if (canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      DogLog.logFault("CANrange distance not updating", AlertType.kError);
    } else {
      DogLog.clearFault("CANrange distance not updating");
    }
    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
    DogLog.forceNt.log("HopperManager/FilteredHopperDistance", filteredDistance);
    DogLog.log("HopperManager/HopperCapacity", hopperCapacity);
    DogLog.forceNt.log("HopperManager/TowerSensor", towerSensorRaw);
  }

  private void setState(HopperState newState) {
    setStateFromRequest(newState);
  }

  private HopperState resolveIdleState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.INTAKING;
    }

    if (operatorWantsStow) {
      return HopperState.IDLE_STOWED;
    }

    if (wantsSafeStow) {
      return HopperState.IDLE_SAFE_KICKER_STOW;
    }

    return HopperState.IDLE_DEPLOYED;
  }

  private HopperState resolveScoreState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.SCORE_AND_INTAKE;
    }

    return HopperState.SCORE;
  }

  private HopperState resolveFeedState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.FEED_AND_INTAKE;
    }

    return HopperState.FEED;
  }

  public void scoreRequest(boolean shouldBeastMode) {
    this.shouldBeastMode = shouldBeastMode;
    setState(resolveScoreState());
  }

  public void scoreRequest() {
    scoreRequest(false);
  }

  public void feedRequest() {
    setState(resolveFeedState());
  }

  public boolean isIntaking() {
    return collector.getState() == CollectorState.INTAKE;
  }

  public boolean isShooting() {
    // You need to actually be in a shooting state
    if (getState() != HopperState.SCORE
        && getState() != HopperState.SCORE_AND_INTAKE
        && getState() != HopperState.FEED
        && getState() != HopperState.FEED_AND_INTAKE) {
      return true;
    }

    return isShootingStrict();
  }

  private boolean isShootingStrict() {
    if (RobotBase.isSimulation()) {
      return !timeout(1.5);
    }
    return towerSensorDebounced;
  }

  public void idleRequest() {
    setState(resolveIdleState());
  }

  public void unjamRequest() {
    setState(HopperState.UNJAMMING);
  }

  public void setDriverWantsEject(boolean wantsEject) {
    driverWantsEject = wantsEject;
  }

  public void setDriverWantsIntake(boolean wantsIntake) {
    wantsSafeStow = false;
    driverWantsIntake = wantsIntake;
  }

  public void setOperatorWantsStow(boolean wantsStow) {
    operatorWantsStow = wantsStow;
  }

  public void setWantsSafeStow(boolean wantsSafeStow) {
    this.wantsSafeStow = wantsSafeStow;
  }

  @Override
  protected void collectInputs() {
    if (RobotBase.isSimulation()) {
      hopperDistance = 20;
      towerSensorRaw =
          switch (getState()) {
            case IDLE_DEPLOYED, IDLE_STOWED, INTAKING -> timeout(5);
            default -> false;
          };
    } else if (DSOptions.USE_TOWER_SENSOR.getAsBoolean()) {
      towerSensorRaw = towerSensor.get();
    } else {
      towerSensorRaw = true;
    }
    towerSensorDebounced = towerSensorDebouncer.calculate(towerSensorRaw);

    if (DSOptions.USE_CANRANGE.get()) {
      hopperDistance = Units.metersToInches(hopperDistanceSignal.getValueAsDouble());
    }
    filteredDistance = hopperFilter.calculate(hopperDistance);

    if (filteredDistance <= HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else if (filteredDistance <= MEDIUM_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.MEDIUM;
    } else {
      hopperCapacity = HopperCapacity.LOW;
    }
  }
}
