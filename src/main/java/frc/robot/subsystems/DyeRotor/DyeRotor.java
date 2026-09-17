package frc.robot.subsystems.DyeRotor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team233.math.MathHelpers;
import com.team233.mechanisms.PowerManaged;
import com.team233.signals.Signals;
import com.team233.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class DyeRotor extends StateMachineSubsystem<RotorState> implements PowerManaged {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> topVelocitySignal;
  private final StatusSignal<AngularVelocity> bottomVelocitySignal;
  private final StatusSignal<Current> topStatorCurrentSignal;
  private final StatusSignal<Current> bottomStatorCurrentSignal;

  private double averageCurrent = 0.0;

  public DyeRotor(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.FEEDER, RotorState.IDLE);
    topMotor.getConfigurator().apply(RotorConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(RotorConfig.BOTTOM_MOTOR_CONFIG);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;

    topVelocitySignal = topMotor.getVelocity(false);
    bottomVelocitySignal = bottomMotor.getVelocity(false);
    topStatorCurrentSignal = topMotor.getStatorCurrent(false);
    bottomStatorCurrentSignal = bottomMotor.getStatorCurrent(false);
    Signals.forDevice(topMotor).addSignals(topVelocitySignal, topStatorCurrentSignal);
    Signals.forDevice(bottomMotor).addSignals(bottomVelocitySignal, bottomStatorCurrentSignal);
  }

  public void shootRequest() {
    setStateFromRequest(RotorState.SHOOT);
  }

  public void intakeRequest() {
    setStateFromRequest(RotorState.INTAKING);
  }

  public void idleRequest() {
    setStateFromRequest(RotorState.IDLE);
  }

  public void ejectRequest() {
    setStateFromRequest(RotorState.EJECT);
  }

  public void ballFillingRequest() {
    setStateFromRequest(RotorState.BALL_FILLING);
  }

  @Override
  protected void afterTransition(RotorState newState) {
    switch (newState) {
      case IDLE -> {
        topMotor.setControl(neutralRequest);
        bottomMotor.setControl(neutralRequest);
      }
      default -> {
        topMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        bottomMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Feeder/Top/VelocityRPM", topVelocitySignal.getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Bottom/VelocityRPM", bottomVelocitySignal.getValueAsDouble() * 60.0);

    averageCurrent =
        MathHelpers.average(
            topStatorCurrentSignal.getValueAsDouble(),
            bottomStatorCurrentSignal.getValueAsDouble());
  }

  public double getAverageCurrent() {
    return averageCurrent;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    topMotor
        .getConfigurator()
        .apply(
            RotorConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
    bottomMotor
        .getConfigurator()
        .apply(
            RotorConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}