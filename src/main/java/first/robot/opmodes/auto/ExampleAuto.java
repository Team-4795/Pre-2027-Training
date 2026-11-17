package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

import first.robot.Robot;

@Autonomous
public class ExampleAuto implements OpMode {
  private final Robot robot;

  public ExampleAuto(Robot r) {
    robot = r;
  }

  /** Called once when this opmode transitions to enabled. */
  @Override
  public void start() {
    // Runs a command when enabled (autos are just big complicated commands)

    // this is REALLY goofy, please don't actually write autos like this
    // we will use PathPlanner or BLine, both will provide ways to create paths + commands easily
    Scheduler.getDefault().schedule(
        Command.race( // joystickDrive never finishes, so race should be used to make sure it runs for 3 secs
            robot.drive.joystickDrive(robot.drive, () -> -2, () -> 0, () -> 0), 
            Command.waitFor(Seconds.of(3)).named("Example Auto/Wait 3 Seconds"))
            .named("Example Auto/Drive 3 Seconds")
            // after 3 secs, the drivebase should be stopped
            .andThen(robot.drive.joystickDrive(robot.drive, () -> 0, () -> 0, () -> 0)).named("Example Auto/Stop"));
  }

  /**
   * This function is called when the opmode is no longer selected on the DS or after an enabled run
   * ends. The object will not be reused after this is called.
   */
  @Override
  public void close() {
    // Stop the robot if the opmode is disabled or deselected
    Scheduler.getDefault().schedule(robot.drive.joystickDrive(robot.drive, () -> 0, () -> 0, () -> 0));
  }
}
