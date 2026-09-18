package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.deploy.DeployConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  private final CANBus canivore = new CANBus("233Canivore");
  private final CANBus rio = CANBus.roboRIO();

  public final DifferentialMechanism<TalonFX> deployDifferentialMechanism =
      new DifferentialMechanism<>(
          TalonFX::new,
          new DifferentialMotorConstants<TalonFXConfiguration>()
              .withCANBusName(rio.getName())
              .withLeaderId(15)
              .withFollowerId(30)
              .withAlignment(MotorAlignmentValue.Opposed)
              .withLeaderInitialConfigs(DeployConfig.LEFT_MOTOR_CONFIG)
              .withFollowerInitialConfigs(DeployConfig.RIGHT_MOTOR_CONFIG)
              .withFollowerUsesCommonLeaderConfigs(true));

  public final TalonFX dyeRotorRoller = new TalonFX(18, rio);
  public final TalonFX dyeRotorRotate = new TalonFX(19, rio);

  public final TalonFX collectorLeftMotor = new TalonFX(20, rio);
  public final TalonFX collectorRightMotor = new TalonFX(21, rio);

  public final TalonFX shooterHoodMotor = new TalonFX(14, rio);
  public final TalonFX shooterFlywheelLeftMotor = new TalonFX(16, rio);
  public final TalonFX shooterFlywheelRightMotor = new TalonFX(17, rio);
  public final TalonFX shooterPivotMotor = new TalonFX(13, rio);
  public final TalonFX shooterIndexMotor = new TalonFX(15, rio);

  public final CANrange hopperCANRange = new CANrange(27, rio);
  public final DigitalInput towerSensor = new DigitalInput(9);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
}