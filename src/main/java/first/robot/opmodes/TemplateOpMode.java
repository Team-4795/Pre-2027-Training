package first.robot.opmodes;

import org.wpilib.framework.OpModeRobot;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.opmode.Utility;

import first.robot.Robot;

// OpMode types
@Utility
// @Teleop
// @Autonomous
public class TemplateOpMode implements OpMode {
  private final Robot robot;

  public TemplateOpMode(Robot r) {
    robot = r;
  }

  /**
   * This function is called periodically while the opmode is selected and the robot is disabled.
   * Code that should only run once when the opmode is selected should go in the opmode constructor.
   */
  @Override
  public void disabledPeriodic() {}

  /** Called once when this opmode transitions to enabled. */
  @Override
  public void start() {
  }

  /**
   * This function is called periodically while the opmode is enabled at the rate returned by {@link
   * OpModeRobot#getPeriod()}.
   */
  @Override
  public void periodic() {}

  /**
   * This function is called asynchronously when the robot disables or switches opmodes while this
   * opmode is enabled. Implementations should stop blocking work promptly.
   */
  @Override
  public void end() {}

  /**
   * This function is called when the opmode is no longer selected on the DS or after an enabled run
   * ends. The object will not be reused after this is called.
   */
  @Override
  public void close() {}
}
