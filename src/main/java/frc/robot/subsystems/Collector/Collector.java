package frc.robot.subsystems.Collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team233.mechanisms.PowerManaged;
import com.team233.signals.Signals;
import com.team233.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class Collector extends StateMachineSubsystem<CollectorState> implements PowerManaged {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;

  public Collector(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKE, CollectorState.IDLE);

    leftMotor.getConfigurator().apply(CollectorConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(CollectorConfig.RIGHT_MOTOR_CONFIG);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);
    Signals.forDevice(leftMotor).addSignals(leftSupplyCurrentSignal);
    Signals.forDevice(rightMotor).addSignals(rightSupplyCurrentSignal);
  }

  public void ejectRequest() {
    setStateFromRequest(CollectorState.EJECT);
  }

  public void shootRequest() {
    setStateFromRequest(CollectorState.SHOOT);
  }

  public boolean hasBeenIntaking() {
    if (getState() == CollectorState.INTAKE && timeout(3)) {
      return true;
    }
    return false;
  }

  public void stopShootingRequest() {
    switch (getState()) {
      case SHOOT -> setStateFromRequest(CollectorState.IDLE);
      default -> {}
    }
  }

  public void intakeRequest() {
    setStateFromRequest(CollectorState.INTAKE);
  }

  public void idleRequest() {
    setStateFromRequest(CollectorState.IDLE);
  }

  @Override
  protected void afterTransition(CollectorState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
      }
      default -> {
        leftMotor.setControl(voltageRequest.withOutput(newState.voltage));
        rightMotor.setControl(voltageRequest.withOutput(newState.voltage));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Collector/Left/SupplyCurrent", leftSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Collector/Right/SupplyCurrent", rightSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Collector/HasBeenIntaking", hasBeenIntaking());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            CollectorConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            CollectorConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}