package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import first.robot.Robot;
import first.robot.mechanisms.ExampleTurret;

public class ExampleCommand implements Command {
    /* Note: don't write separate classes for commands unless it makes sense.
     Regular coroutine logic can handle most cases.
     Only write classes for the really complicated/weird commands.
     */

    // this is how you could access mechanisms
    private ExampleTurret turret = Robot.getInstance().turret;

    /** 
     * This method is called when the command is run.
     * You can use loops, coroutine.wait/fork/await (etc).
     * With the coroutine, you can run any other command, including mechanism commands
     */
    @Override
    public void run(Coroutine coroutine) {
        // No loop, just logs something, waits, then logs something else and sets turret goal
        Logger.recordOutput("Example Command", "Hi :)");
        coroutine.wait(Seconds.of(5));
        Logger.recordOutput("Example Command", "Hi again :D");
        coroutine.fork(turret.setGoal(0.5));
    }

    /**
     * This method sets the command's name.
     */
    @Override
    public String name() {
        return "Example Command";
    }

    /**
     * This method sets the command's requirements.
     */
    @Override
    public Set<Mechanism> requirements() {
        // return Set.of(); // no requirements
        return Set.of(turret); // requires turret
    }
}
