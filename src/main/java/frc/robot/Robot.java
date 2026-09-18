package frc.robot;

import com.team233.Base233Robot;
import com.team233.GlobalConfig;
import com.team233.controller.ControllerBindings;
import com.team233.math.PoseErrorTolerance;
import com.team233.trailblazer.Trailblazer;
import com.team233.trailblazer.followers.PidPathFollower;
import com.team233.trailblazer.trackers.HeuristicPathTracker;
import com.team233.util.FieldUtil;
import com.team233.util.FmsUtil;
import dev.doglog.DogLog;
import frc.robot.autos.Autos;
import frc.robot.autos.BumpCrossingFollower;
import frc.robot.cluster_map.ClusterMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import frc.robot.deploy.Deploy;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.imu.Imu;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.DyeRotor.DyeRotor;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Vision.CameraConfigs;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.limelight.Limelight;
import frc.robot.subsystems.Vision.limelight.LimelightState;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class Robot extends Base233Robot {
  private final Hardware hardware = new Hardware();

  private final Limelight frontLimelight =
      new Limelight("front", LimelightState.TAGS, CameraConfigs.FRONT);
  private final Limelight backLimelight =
      new Limelight("back", LimelightState.TAGS, CameraConfigs.BACK);
  private final HealthManager health =
      new HealthManager(frontLimelight, backLimelight);

  private final Imu imu = new Imu(hardware.drivetrain);

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new BumpCrossingFollower(
              new PidPathFollower(
                  new PIDController(3.5, 0, 0),
                  new PIDController(
                      Swerve.ORIGINAL_HEADING_PID.getP(),
                      Swerve.ORIGINAL_HEADING_PID.getI(),
                      Swerve.ORIGINAL_HEADING_PID.getD())),
              imu.bumpCrossingTracker));

  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);

  private final Shooter shooter =
      new Shooter(
          hardware.shooterFlywheelLeftMotor,
          hardware.shooterFlywheelRightMotor,
          hardware.shooterPivotMotor,
          hardware.shooterIndexMotor,
          hardware.shooterHoodMotor);
  private final Collector collector = new Collector(hardware.collectorLeftMotor, hardware.collectorRightMotor);
  private final Deploy deploy = new Deploy(hardware.deployDifferentialMechanism);
  private final Vision vision =
      new Vision(imu, frontLimelight, backLimelight);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);
  private final DyeRotor dyeRotor = new DyeRotor(hardware.dyeRotorRoller, hardware.dyeRotorRotate);

  private final ClusterMap clusterMap = new ClusterMap(localization, swerve, frontLimelight);
  private final HubActivity hubActivity = new HubActivity();

  private final PowerManager powerManager =
      new PowerManager(shooter, collector, deploy, dyeRotor, swerve);
  private final HopperManager hopperManager =
      new HopperManager(
          deploy, collector, dyeRotor, hardware.hopperCANRange, hardware.towerSensor);

  private final RobotManager robotManager =
      new RobotManager(
          hopperManager,
          localization,
          swerve,
          shooter,
          vision,
          hardware.driverController,
          health,
          hubActivity,
          trailblazer,
          clusterMap,
          hardware,
          powerManager);

  @SuppressWarnings("unused") // Registers itself as a subsystem
  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {

    // Reuse the SwerveDriveState from Swerve for IMU
    imu.setDriveStateSupplier(swerve::getDriveState);

    finalizeInit();

    if (GlobalConfig.IS_DEVELOPMENT) {
      FieldUtil.debugLogFieldZones();
    }

    if (RobotBase.isSimulation()) {
      try {
        var docsDir = Path.of(System.getProperty("user.dir")).resolve("../docs");
        Files.writeString(
            docsDir.resolve("feeding_obstructions.svg"), FieldUtil.FEEDING_OBSTRUCTIONS.toSvg());
      } catch (IOException e) {
        throw new RuntimeException("Failed to write field obstacles SVG", e);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (FeatureFlags.CLAMPED_AUTO_POINTS.getAsBoolean() && !FmsUtil.isRedAlliance()) {
      DogLog.logFault("Clamped auto points are enabled but current alliance is blue");
    } else {
      DogLog.clearFault("Clamped auto points are enabled but current alliance is blue");
    }
  }

  @Override
  protected void configureBindings() {
    var driver =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.driverController);
    var operator =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.operatorController);

    driver.back().onPress(localization::zeroGyro);

    driver
        .leftTrigger()
        .onPress(() -> hopperManager.setDriverWantsIntake(true))
        .onRelease(() -> hopperManager.setDriverWantsIntake(false));

    driver
        .rightTrigger()
        .onPress(robotManager::prepareScoreOrFeedRequest)
        .onRelease(robotManager::idleRequest);

    driver.rightBumper().onPress(robotManager::idleRequest);

    driver
        .leftBumper()
        .onPress(() -> hopperManager.setDriverWantsEject(true))
        .onRelease(() -> hopperManager.setDriverWantsEject(false));

    operator.start().onPress(() -> hopperManager.deploy.homingRequest());

    operator.x().onPress(robotManager::unjamRequest).onRelease(robotManager::idleRequest);

    // operator.b().onPress(robotManager::prepareFeedRequest).onRelease(robotManager::idleRequest);

    operator.b().onPress(deploy::fixDifferentialDesyncRequest).onRelease(deploy::intakeRequest);

    operator
        .rightTrigger()
        .onPress(robotManager::prepareScoreRequest)
        .onRelease(robotManager::idleRequest);

    operator
        .leftTrigger()
        .onPress(robotManager::stowDeployRequest)
        .onRelease(robotManager::cancelStowDeployRequest);

    operator
        .leftBumper()
        .onPress(() -> robotManager.powerManager.prioritizeIntakeRequest())
        .onRelease(() -> robotManager.powerManager.idleRequest());
    operator
        .rightBumper()
        .onPress(() -> robotManager.setTrenchOverrideRequest(true))
        .onRelease(() -> robotManager.setTrenchOverrideRequest(false));
  }
}