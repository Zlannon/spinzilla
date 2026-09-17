package com.team233;

import edu.wpi.first.wpilibj.RobotBase;

public final class GlobalConfig {
  public static final boolean IS_DEVELOPMENT = RobotBase.isSimulation();

  private GlobalConfig() {}
}
