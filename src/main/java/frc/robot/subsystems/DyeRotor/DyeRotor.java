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

  private final TalonFX dyeRotorRoller;
  private final TalonFX dyeRotorRotate;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> rollerVelocitySignal;
  private final StatusSignal<AngularVelocity> rotateVelocitySignal;
  private final StatusSignal<Current> rollerStatorCurrentSignal;
  private final StatusSignal<Current> rotateStatorCurrentSignal;

  private double averageCurrent = 0.0;

  public DyeRotor(TalonFX dyeRotorRoller, TalonFX dyeRotorRotate) {
    super(SubsystemPriority.FEEDER, RotorState.IDLE);
    dyeRotorRoller.getConfigurator().apply(RotorConfig.ROLLER_MOTOR_CONFIG);
    dyeRotorRotate.getConfigurator().apply(RotorConfig.ROTATE_MOTOR_CONFIG);
    this.dyeRotorRoller = dyeRotorRoller;
    this.dyeRotorRotate = dyeRotorRotate;

    rollerVelocitySignal = dyeRotorRoller.getVelocity(false);
    rotateVelocitySignal = dyeRotorRotate.getVelocity(false);
    rollerStatorCurrentSignal = dyeRotorRoller.getStatorCurrent(false);
    rotateStatorCurrentSignal = dyeRotorRotate.getStatorCurrent(false);
    Signals.forDevice(dyeRotorRoller).addSignals(rollerVelocitySignal, rollerStatorCurrentSignal);
    Signals.forDevice(dyeRotorRotate).addSignals(rotateVelocitySignal, rotateStatorCurrentSignal);
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
        dyeRotorRoller.setControl(neutralRequest);
        dyeRotorRotate.setControl(neutralRequest);
      }
      default -> {
        dyeRotorRoller.setControl(voltageRequest.withOutput(newState.getVoltage()));
        dyeRotorRotate.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("DyeRotor/Roller/VelocityRPM", rollerVelocitySignal.getValueAsDouble() * 60.0);
    DogLog.log("DyeRotor/Rotate/VelocityRPM", rotateVelocitySignal.getValueAsDouble() * 60.0);

    averageCurrent =
        MathHelpers.average(
            rollerStatorCurrentSignal.getValueAsDouble(),
            rotateStatorCurrentSignal.getValueAsDouble());
  }

  public double getAverageCurrent() {
    return averageCurrent;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    dyeRotorRoller
        .getConfigurator()
        .apply(
            RotorConfig.ROLLER_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
    dyeRotorRotate
        .getConfigurator()
        .apply(
            RotorConfig.ROTATE_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}