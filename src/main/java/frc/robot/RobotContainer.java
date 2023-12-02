// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveRobot;

import frc.robot.commands.EmergAuto;

import frc.robot.commands.Intake;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeThing;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.EmergAuto;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final IntakeThing m_IntakeThing = new IntakeThing();

  private SendableChooser<Command> autoChooser;
  public static HashMap<String, Command> eventMap;
  private Autos autos;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController =

      new Joystick(OperatorConstants.kDriverControllerPort);
  private final PS4Controller m_operatorController = 
      new PS4Controller(OperatorConstants.kCoDriverControllerPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();
    // Configure the trigger bindings
    m_Drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    configureBindings();
    autoInit();
    SmartDashboard.putNumber("heading", 0);
  }

  private void autoInit() {
    autos = Autos.getInstance();
    eventMap.put("stop", new InstantCommand(m_Drivetrain::stop, m_Drivetrain));
    eventMap.put("run Intake", new InstantCommand(m_IntakeThing::runAtDefault, m_IntakeThing));
    autos.autoInit(autoChooser, eventMap, m_Drivetrain, m_IntakeThing);
    SmartDashboard.putData("autoChooser", autoChooser);
    SmartDashboard.putBoolean("is autoInit running", true);
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
    m_Drivetrain.setDefaultCommand(new DriveRobot(m_Drivetrain, m_driverController));
    m_IntakeThing.setDefaultCommand(new Intake(m_IntakeThing, m_operatorController));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
        while (m_driverController.getTrigger()) {}
      
    }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

  }

}