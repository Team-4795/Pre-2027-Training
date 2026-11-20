// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandXboxController;

import first.robot.mechanisms.ExampleTurret;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.drive.Drive;
import first.robot.mechanisms.drive.GyroIO;
import first.robot.mechanisms.drive.ModuleIOSim;

public class Robot extends LoggedOpModeRobot {
  // Controllers
  public CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_PORT);
  public CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_PORT);

  // Mechanisms
  public final ExampleTurret turret;
  public final Drive drive;
  public final Shooter shooter;

  // Robot instance
  private static Robot instance;

  /* 1c. Create the Shooter object
  Declare your shooter object, then create a new Shooter object in the constructor.
   */

  public Robot() {
    instance = this;
    Logger.addDataReceiver(new NT4Publisher());

    // Initialize simulated mechanisms
    turret = new ExampleTurret();
    shooter = new Shooter();

    // dw about this one too much
    drive = Drive.initialize(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
    
    // Start Advantagekit logger
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    // Call mechanisms' perodic methods every loop
    turret.periodic();
    drive.periodic();
    shooter.periodic();
    /* 1d. Call the Shooter periodic method
    Call your shooter's periodic() method so that sim and logging update.
     */

    // Run commands and stuff
    Scheduler.getDefault().run();
  }

  public static Robot getInstance() {
    return instance;
  }
}
