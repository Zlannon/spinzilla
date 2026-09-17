package frc.robot.subsystems.Vision;

import com.team233.math.MathHelpers;
import com.team233.util.state_machines.StateMachineSubsystem;
import com.team233.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.Imu;
import frc.robot.subsystems.Vision.limelight.Limelight;
import frc.robot.subsystems.Vision.limelight.LimelightState;
import frc.robot.util.scheduling.SubsystemPriority;

public class Vision extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingHubTagDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Imu imu;
  private final Limelight frontLimelight;
  private final Limelight backLimelight;

  private OptionalTagResult shooterResult = new OptionalTagResult();
  private OptionalTagResult leftResult = new OptionalTagResult();
  private OptionalTagResult rightResult = new OptionalTagResult();

  private double robotHeading;

  private double robotAngularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;

  private boolean seeingHubTags = false;

  public Vision(
      Imu imu,
      Limelight frontLimelight,
      Limelight backLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.frontLimelight = frontLimelight;
    this.backLimelight = backLimelight;
  }

  @Override
  protected VisionState getNextState(VisionState currentState) {
    return switch (currentState) {
      case HUB_TAGS, WAITING_FOR_HUB_TAGS -> {
        if (seeingHubTags) {
          yield VisionState.HUB_TAGS;
        }
        yield VisionState.WAITING_FOR_HUB_TAGS;
      }
      default -> currentState;
    };
  }

  public void setRobotVelocity(double velocity) {
    frontLimelight.setRobotVelocity(velocity);
    backLimelight.setRobotVelocity(velocity);
  }

  @Override
  protected void collectInputs() {
    robotAngularVelocity = imu.getRobotAngularVelocity();

    shooterResult = frontLimelight.getTagResult();
    leftResult = backLimelight.getTagResult();

    if (shooterResult.isPresent() || leftResult.isPresent() || rightResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }

    seeingHubTags =
        seeingHubTagDebouncer.calculate(
            frontLimelight.seeingHubTag()
                || backLimelight.seeingHubTag());
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = MathHelpers.angleModulus(robotHeading);
    // Send IMU data to all limelights
    frontLimelight.sendImuData(this.robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
    backLimelight.sendImuData(this.robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
  }

  public OptionalTagResult getfrontLimelightTagResult() {
    return shooterResult;
  }

  public OptionalTagResult getbackLimelightTagResult() {
    return leftResult;
  }

  public OptionalTagResult getRightLimelightTagResult() {
    return rightResult;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void tagsRequest() {
    setStateFromRequest(VisionState.TAGS);
  }

  public void hubTagsRequest() {
    if (getState() == VisionState.WAITING_FOR_HUB_TAGS || getState() == VisionState.HUB_TAGS) {
      return;
    }

    setStateFromRequest(VisionState.WAITING_FOR_HUB_TAGS);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        frontLimelight.setState(LimelightState.TAGS);
        backLimelight.setState(LimelightState.TAGS);
      }
      case HUB_TAGS -> {
        frontLimelight.setState(LimelightState.HUB_TAGS);
        backLimelight.setState(LimelightState.HUB_TAGS);
      }
      case WAITING_FOR_HUB_TAGS -> {
        frontLimelight.setState(LimelightState.TAGS);
        backLimelight.setState(LimelightState.TAGS);
      }
      default -> {}
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    DogLog.log("Vision/SeeingTag", seeingTag);
  }
}
