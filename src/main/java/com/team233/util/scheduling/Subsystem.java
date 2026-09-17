package com.team233.util.scheduling;

public interface Subsystem {
  SubsystemPriorityBase getPriority();

  void periodic();
}
