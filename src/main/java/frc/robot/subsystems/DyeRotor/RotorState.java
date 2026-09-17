package frc.robot.subsystems.DyeRotor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum RotorState {
  SHOOT(10),
  BALL_FILLING(1),
  INTAKING(-1),
  EJECT(-6),
  IDLE(0);

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  RotorState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Feeder/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}