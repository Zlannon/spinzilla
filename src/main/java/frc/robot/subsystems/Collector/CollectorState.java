package frc.robot.subsystems.Collector;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum CollectorState {
  INTAKE(12),
  EJECT(-12),
  IDLE(0),
  SHOOT(5);

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  CollectorState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Collector/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}