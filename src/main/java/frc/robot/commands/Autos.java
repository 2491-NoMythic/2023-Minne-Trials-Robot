// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathRamsete;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeThing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import java.nio.file.Path;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.PathPoint;
import frc.robot.subsystems.IntakeThing;

import java.util.function.BiConsumer;

public final class Autos {
  private Drivetrain drivetrain;
  public static RamseteAutoBuilder autoBuilder;
  private HashMap<String, Command> eventMap;
  public RamseteController controller = new RamseteController();

  private IntakeThing intake;
  private static Autos autos;

  private void Autos() {}

  public static Autos getInstance() {
    if (autos == null) {
      autos = new Autos();
    }
    return autos; 
  }

  public void autoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, Drivetrain drivetrain, IntakeThing intake) {
    this.drivetrain = drivetrain;
    this.eventMap = eventMap;
    this.intake = intake;


    //time to create the autobuilder!! run this when autos are initialized


    Autos.autoBuilder = new RamseteAutoBuilder(
      drivetrain::getPose, 
      drivetrain::resetOdometry, 
      controller, //nothings been done to this, it's just been created
      drivetrain.kinematics, 
      drivetrain::driveLR, // apparently an "output consumer" is a fancy name for a drive command that uses Left and Right speeds
      eventMap, 
      drivetrain);
    
    autoChooser.addOption("test1", test1());
  //format for adding an Auto:
  //autoChooser.addOption("name that will appear on smartDashboard", nameOfMethodBelow);
  //then write this OUTSIDE of AutoInit:
  //public SequentialCommandGroup nameOfAuto() {
  //  return new SequentialCommandGroup(
  //    autoBuilder.fullAuto(nameListedBelow),
  //    new CommandToRunAfterAfterAutoRuns,
  //    new AnotherCommandToRun))  
  //}
}
  public SequentialCommandGroup test1() {
    return new SequentialCommandGroup(
      autoBuilder.fullAuto(MinneTrialstest1),
      new InstantCommand(intake::runAtDefault, intake)
    );
  }

  static List<PathPlannerTrajectory> MinneTrialstest1 = PathPlanner.loadPathGroup("MinneTrialsTest1", new PathConstraints(2.5, 1.75));
}