package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team233.mechanisms.PowerManaged;
import com.team233.signals.Signals;
import com.team233.simkit.SimKit;
import com.team233.util.state_machines.StateMachineSubsystem;
import com.team233.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> implements PowerManaged {
  private static double distanceToScoringRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_SCORE_RPM.get(distance);
  }

  private static double distanceToFeedingRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_FEEDING_RPM.get(distance);
  }

  private final TalonFX shooterFlywheelLeftMotor;
  private final TalonFX shooterFlywheelRightMotor;
  public final TalonFX shooterPivotMotor;
  public final TalonFX shooterIndexMotor;
  public final TalonFX shooterHoodMotor;

  private final Follower flywheelLeftFollower;

  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  private final StatusSignal<AngularVelocity> flywheelLeftVelocitySignal;
  private final StatusSignal<AngularVelocity> flywheelRightVelocitySignal;
  private final StatusSignal<AngularVelocity> pivotVelocitySignal;
  private final StatusSignal<AngularVelocity> indexVelocitySignal;
  private final StatusSignal<Voltage> flywheelLeftVoltageSignal;
  private final StatusSignal<Voltage> flywheelRightVoltageSignal;
  private final StatusSignal<Voltage> pivotVoltageSignal;
  private final StatusSignal<Voltage> indexVoltageSignal;
  private final StatusSignal<Voltage> flywheelRightSupplyVoltageSignal;
  private final StatusSignal<Current> flywheelLeftSupplyCurrentSignal;
  private final StatusSignal<Current> flywheelRightSupplyCurrentSignal;
  private final StatusSignal<Current> pivotSupplyCurrentSignal;
  private final StatusSignal<Current> indexSupplyCurrentSignal;
  private final StatusSignal<Current> flywheelRightTorqueCurrentSignal;

  private double flywheelLeftVoltage = 0;
  private double flywheelRightVoltage = 0;
  private double pivotVoltage = 0;
  private double indexVoltage = 0;
  private double flywheelRightSupplyVoltage = 0;
  private double flywheelLeftSupplyCurrent = 0;
  private double flywheelRightSupplyCurrent = 0;
  private double pivotSupplyCurrent = 0;
  private double indexSupplyCurrent = 0;
  private double flywheelRightTorqueCurrent = 0;

  private double scoreDistance = 0;
  private double feedDistance = 0;

  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double flywheelLeftMotorRpm = 0;
  private double flywheelRightMotorRpm = 0;
  private double pivotMotorRpm = 0;
  private double indexMotorRpm = 0;
  private double feederCurrent = 0.0;
  private double feederBasedFeedForward = 0.0;
  private boolean hopperFull = false;

  private boolean atGoal = false;
  private boolean atGoalDebounced = false;

  // Debouncer for delay between shots at 15 bps
  private final Debouncer atGoalDebouncer = new Debouncer(1.0 / 15.0, DebounceType.kFalling);

  public Shooter(
      TalonFX shooterFlywheelLeftMotor,
      TalonFX shooterFlywheelRightMotor,
      TalonFX shooterPivotMotor,
      TalonFX shooterIndexMotor,
      TalonFX shooterHoodMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    shooterFlywheelLeftMotor.getConfigurator().apply(ShooterConfig.FLYWHEEL_LEFT_MOTOR_CONFIGS);
    shooterFlywheelRightMotor.getConfigurator().apply(ShooterConfig.FLYWHEEL_RIGHT_MOTOR_CONFIG);
    shooterPivotMotor.getConfigurator().apply(ShooterConfig.PIVOT_MOTOR_CONFIG);
    shooterIndexMotor.getConfigurator().apply(ShooterConfig.INDEX_MOTOR_CONFIG);

    TunablePid.register("Shooter/flywheelLeft", shooterFlywheelLeftMotor, ShooterConfig.FLYWHEEL_LEFT_MOTOR_CONFIGS);
    TunablePid.register("Shooter/flywheelRight", shooterFlywheelRightMotor, ShooterConfig.FLYWHEEL_RIGHT_MOTOR_CONFIG);
    TunablePid.register(
        "Shooter/pivot", shooterPivotMotor, ShooterConfig.PIVOT_MOTOR_CONFIG);
    TunablePid.register(
        "Shooter/index", shooterIndexMotor, ShooterConfig.INDEX_MOTOR_CONFIG);

    this.shooterFlywheelLeftMotor = shooterFlywheelLeftMotor;
    this.shooterFlywheelRightMotor = shooterFlywheelRightMotor;
    this.shooterPivotMotor = shooterPivotMotor;
    this.shooterIndexMotor = shooterIndexMotor;
    this.shooterHoodMotor = shooterHoodMotor;

    // Only the left flywheel motor follows the right flywheel motor
    this.flywheelLeftFollower = new Follower(shooterFlywheelRightMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    shooterFlywheelLeftMotor.setControl(flywheelLeftFollower);

    flywheelLeftVelocitySignal = shooterFlywheelLeftMotor.getVelocity(false);
    flywheelRightVelocitySignal = shooterFlywheelRightMotor.getVelocity(false);
    pivotVelocitySignal = shooterPivotMotor.getVelocity(false);
    indexVelocitySignal = shooterIndexMotor.getVelocity(false);
    flywheelLeftVoltageSignal = shooterFlywheelLeftMotor.getMotorVoltage(false);
    flywheelRightVoltageSignal = shooterFlywheelRightMotor.getMotorVoltage(false);
    pivotVoltageSignal = shooterPivotMotor.getMotorVoltage(false);
    indexVoltageSignal = shooterIndexMotor.getMotorVoltage(false);
    flywheelRightSupplyVoltageSignal = shooterFlywheelRightMotor.getSupplyVoltage(false);
    flywheelLeftSupplyCurrentSignal = shooterFlywheelLeftMotor.getSupplyCurrent(false);
    flywheelRightSupplyCurrentSignal = shooterFlywheelRightMotor.getSupplyCurrent(false);
    pivotSupplyCurrentSignal = shooterPivotMotor.getSupplyCurrent(false);
    indexSupplyCurrentSignal = shooterIndexMotor.getSupplyCurrent(false);
    flywheelRightTorqueCurrentSignal = shooterFlywheelRightMotor.getTorqueCurrent(false);

    Signals.forDevice(shooterFlywheelRightMotor)
        .addSignals(flywheelLeftVelocitySignal, flywheelLeftVoltageSignal, flywheelLeftSupplyCurrentSignal);
    Signals.forDevice(shooterFlywheelRightMotor)
        .addSignals(
            flywheelRightVelocitySignal,
            flywheelRightVoltageSignal,
            flywheelRightSupplyVoltageSignal,
            flywheelRightSupplyCurrentSignal,
            flywheelRightTorqueCurrentSignal);
    Signals.forDevice(shooterPivotMotor)
        .addSignals(
            pivotVelocitySignal, pivotVoltageSignal, pivotSupplyCurrentSignal);
    Signals.forDevice(shooterIndexMotor)
        .addSignals(
            indexVelocitySignal, indexVoltageSignal, indexSupplyCurrentSignal);
  }

  public void prepareScoreRequest(double distance) {
    this.scoreDistance = distance;
    if (getState() != ShooterState.SCORE) {
      setStateFromRequest(ShooterState.PREPARE_SCORE);
    }
  }

  public void scoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState.SCORE);
  }

  public void prepareFeedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.PREPARE_FEED);
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.FEED);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  public void updateHopperState(double feederCurrent, boolean hopperFull) {
    this.feederCurrent = feederCurrent;
    this.hopperFull = hopperFull || !DSOptions.USE_CANRANGE.getAsBoolean();
  }

  @Override
  protected void whileInState(ShooterState state) {
    DogLog.log("Shooter/flywheelLeft/RPM", flywheelLeftMotorRpm);
    DogLog.log("Shooter/flywheelRight/RPM", flywheelRightMotorRpm);
    DogLog.log("Shooter/pivot/RPM", pivotMotorRpm);
    DogLog.log("Shooter/index/RPM", indexMotorRpm);
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/FeederBasedFeedForward", feederBasedFeedForward);
    DogLog.log("Shooter/AtGoal", atGoal);

    DogLog.log("Shooter/flywheelLeft/SupplyCurrent", flywheelLeftSupplyCurrent);
    DogLog.log("Shooter/flywheelRight/SupplyCurrent", flywheelRightSupplyCurrent);
    DogLog.log("Shooter/pivot/SupplyCurrent", pivotSupplyCurrent);
    DogLog.log("Shooter/index/SupplyCurrent", indexSupplyCurrent);

    switch (state) {
      case IDLE -> {
        var setpoint = ShooterConfig.IDLE_RPM / 60.0;
        shooterFlywheelRightMotor.setControl(velocityRequest.withVelocity(setpoint).withFeedForward(0.0));
        DogLog.log("Shooter/RpmSetpoint", ShooterConfig.IDLE_RPM);
      }
      case PREPARE_SCORE -> {
        var setpoint = shootingRpm / 60.0;
        shooterFlywheelRightMotor.setControl(
            velocityRequest.withVelocity(setpoint).withFeedForward(feederBasedFeedForward));
        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case SCORE -> {
        var setpoint = shootingRpm / 60.0;
        shooterFlywheelRightMotor.setControl(
            velocityRequest.withVelocity(setpoint).withFeedForward(feederBasedFeedForward));
        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case PREPARE_FEED -> {
        var setpoint = feedingRpm / 60.0;
        shooterFlywheelRightMotor.setControl(
            velocityRequest.withVelocity(setpoint).withFeedForward(feederBasedFeedForward));
        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
      case FEED -> {
        var setpoint = feedingRpm / 60.0;
        shooterFlywheelRightMotor.setControl(
            velocityRequest.withVelocity(setpoint).withFeedForward(feederBasedFeedForward));
        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToScoringRpm(scoreDistance));
    feedingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToFeedingRpm(feedDistance));

    if (DSOptions.PIT_FUNCTIONALITY.getAsBoolean()) {
      shootingRpm = ShooterConfig.PIT_FUNCTIONALITY_RPM;
      feedingRpm = ShooterConfig.PIT_FUNCTIONALITY_RPM;
    }

    flywheelLeftMotorRpm = flywheelLeftVelocitySignal.getValueAsDouble() * 60.0;
    flywheelRightMotorRpm = flywheelRightVelocitySignal.getValueAsDouble() * 60.0;
    pivotMotorRpm = pivotVelocitySignal.getValueAsDouble() * 60.0;
    indexMotorRpm = indexVelocitySignal.getValueAsDouble() * 60.0;

    flywheelLeftVoltage = flywheelLeftVoltageSignal.getValueAsDouble();
    flywheelRightVoltage = flywheelRightVoltageSignal.getValueAsDouble();
    pivotVoltage = pivotVoltageSignal.getValueAsDouble();
    indexVoltage = indexVoltageSignal.getValueAsDouble();
    flywheelRightSupplyVoltage = flywheelRightSupplyVoltageSignal.getValueAsDouble();
    flywheelLeftSupplyCurrent = flywheelLeftSupplyCurrentSignal.getValueAsDouble();
    flywheelRightSupplyCurrent = flywheelRightSupplyCurrentSignal.getValueAsDouble();
    pivotSupplyCurrent = pivotSupplyCurrentSignal.getValueAsDouble();
    indexSupplyCurrent = indexSupplyCurrentSignal.getValueAsDouble();
    flywheelRightTorqueCurrent = flywheelRightTorqueCurrentSignal.getValueAsDouble();

    atGoal = calculateAtGoal();
    atGoalDebounced = atGoalDebouncer.calculate(atGoal);

    switch (getState()) {
      case SCORE, FEED -> {
        if (!timeout(0.7) && hopperFull) {
          feederBasedFeedForward =
              Math.max(
                  ShooterConfig.FEEDER_CURRENT_TO_SHOOTER_FEED_FORWARD.get(feederCurrent),
                  ShooterConfig.FULL_HOPPER_INITIAL_FF.getAsDouble());
        } else if (!timeout(0.1)) {
          feederBasedFeedForward =
              Math.max(
                  ShooterConfig.FEEDER_CURRENT_TO_SHOOTER_FEED_FORWARD.get(feederCurrent),
                  ShooterConfig.LOW_HOPPER_INITIAL_FFF.getAsDouble());
        } else {
          feederBasedFeedForward =
              ShooterConfig.FEEDER_CURRENT_TO_SHOOTER_FEED_FORWARD.get(feederCurrent);
        }
      }
      default -> {
        feederBasedFeedForward =
            ShooterConfig.FEEDER_CURRENT_TO_SHOOTER_FEED_FORWARD.get(feederCurrent);
      }
    }
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean atGoalDebounced() {
    return atGoalDebounced;
  }

  private boolean calculateAtGoal() {
    return switch (getState()) {
      case IDLE -> false;
      case PREPARE_SCORE ->
          MathUtil.isNear(flywheelRightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE);
      case SCORE ->
          MathUtil.isNear(
              flywheelRightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_ACTIVELY_SHOOTING);
      case PREPARE_FEED, FEED ->
          MathUtil.isNear(flywheelRightMotorRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING);
      default -> true;
    };
  }

  @Override
  public void simulationPeriodic() {
    var shooterSimulation =
        SimKit.velocityMechanism(
            "shooter",
            (mechanism) ->
                mechanism
                    .addMotor(shooterFlywheelLeftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(shooterFlywheelRightMotor, ChassisReference.CounterClockwise_Positive)
                    .addMotor(shooterPivotMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(shooterIndexMotor, ChassisReference.CounterClockwise_Positive));

    shooterSimulation.update();
  }

  public double getScoreTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_TOF_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_SCORE_TOF.get(distance);
  }

  public double getFeedTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_TOF_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_FEED_TOF.get(distance);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    shooterFlywheelLeftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.FLYWHEEL_LEFT_MOTOR_CONFIGS.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    shooterFlywheelRightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.FLYWHEEL_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    shooterPivotMotor
        .getConfigurator()
        .apply(
            ShooterConfig.PIVOT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    shooterIndexMotor
        .getConfigurator()
        .apply(
            ShooterConfig.INDEX_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}