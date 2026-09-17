package frc.robot.subsystems.Vision;

import com.team233.config.CameraConfig;
import com.team233.config.LimelightModel;
import edu.wpi.first.math.util.Units;

public class CameraConfigs {

  public static final CameraConfig FRONT =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          // Forward
          Units.inchesToMeters(0.0),
          // Right
          Units.inchesToMeters(0.0),
          // Up
          Units.inchesToMeters(0.0),
          // Pitch
          0.0,
          // Yaw
          0.0,
          // Roll
          0.0);

  public static final CameraConfig BACK =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          // Forward
          Units.inchesToMeters(0.0),
          // Right
          Units.inchesToMeters(0.0),
          // Up
          Units.inchesToMeters(0.0),
          // Pitch
          0.0,
          // Yaw
          0.0,
          // Roll
          0.0);

  private CameraConfigs() {}
}
