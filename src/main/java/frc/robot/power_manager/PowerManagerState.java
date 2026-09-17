package frc.robot.power_manager;

public enum PowerManagerState {
  AUTO_FIRST_SEGMENT(50, 80, 15, 40, 10, 10, 70),
  PRIORITIZE_INTAKE(50, 80, 15, 40, 10, 10, 20),
  IDLE(50, 25, 25, 40, 10, 10, 35),
  SCORING(50, 20, 15, 40, 20, 20, 20),
  SCORING_FAR(50, 20, 15, 40, 10, 10, 20),
  BEAST_MODE(50, 10, 15, 40, 30, 30, 20),
  FEEDING(35, 20, 15, 40, 30, 30, 35),
  FEEDING_FAR(35, 20, 15, 40, 15, 15, 30),

  TURBO_MODE(30, 5, 18, 10, 10, 10, 70);

  final double shooterSupplyCurrent;
  final double collectorSupplyCurrent;
  final double deploySupplyCurrent;
  final double shooterHoodSupplyCurrent;
  final double dyeRotorSupplyCurrent;
  final double swerveSupplyCurrent;

  PowerManagerState(
      double shooterSupplyCurrent,
      double collectorSupplyCurrent,
      double deploySupplyCurrent,
      double shooterHoodSupplyCurrent,
      double dyeRotorSupplyCurrent,
      double conveyorSupplyCurrent,
      double swerveSupplyCurrent) {
    this.shooterSupplyCurrent = shooterSupplyCurrent;
    this.collectorSupplyCurrent = collectorSupplyCurrent;
    this.deploySupplyCurrent = deploySupplyCurrent;
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent;
    this.dyeRotorSupplyCurrent = dyeRotorSupplyCurrent;
    this.swerveSupplyCurrent = swerveSupplyCurrent;
  }
}
