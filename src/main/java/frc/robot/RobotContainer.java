// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveRobot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SparkSystem;
import frc.robot.commands.DoSparkSystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.commands.DoSparkSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final SparkSystem m_SparkSystem = new SparkSystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController =

      new Joystick(OperatorConstants.kDriverControllerPort);
  private final PS4Controller opController = new PS4Controller(OperatorConstants.kOpControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //new Trigger;
    new Trigger(opController::getL1Button).whileTrue(new DoSparkSystem(m_SparkSystem));
  



m_Drivetrain.setDefaultCommand(new DriveRobot(m_Drivetrain, m_driverController));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
        while (m_driverController.getTrigger()) {}
      
    }

  
   // Use this to pass the autonomous command to the main {@link Robot} class.
   
    //@return the command to run in autonomous
    public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  }