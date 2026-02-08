package us.kilroyrobotics.subsystems.intake;

public enum IntakeEvent {
  /** Nothing should be changing */
  NONE,

  /** Request for the intake to extend out */
  EXTEND,

  /** Request for the intake roller to spin up */
  START_INTAKING,

  /** Request for the intake roller to spin down */
  STOP_INTAKING,

  /** Request for the intake to retract */
  RETRACT,

  /** 
   * Request for the intake to agitate
   * 
   * <p>This should be triggered when the robot is launching fuel
   */
  AGITATE
}
