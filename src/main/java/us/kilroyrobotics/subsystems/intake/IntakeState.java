package us.kilroyrobotics.subsystems.intake;

public enum IntakeState {
  /**
   * The intake is retracted and inside of the robot perimeter.
   *
   * <p>[Initial State]
   */
  RETRACTED,

  /** The intake is in the process of extending outside of the robot perimeter to intake fuel. */
  EXTENDING,

  /** The intake is extended outside of the robot perimeter and ready to intake fuel. */
  EXTENDED,

  /** The intake rollers are active and intaking fuel. */
  INTAKING,

  /**
   * The intake is in the process of retracting inside of the robot perimeter.
   *
   * <p>This is a smooth motion, not to be confused with {@link IntakeState#AGITATING AGITATING}.
   */
  RETRACTING,

  /**
   * The intake is in the process of retracting inside of the robot perimeter while the intake
   * rollers are running and fuel is being launched.
   *
   * <p>This is a rough motion, not to be confused with {@link IntakeState#RETRACTING RETRACTING}
   */
  AGITATING
}
