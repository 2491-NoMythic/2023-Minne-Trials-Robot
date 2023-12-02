// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

public class DriveForwardMeters extends CommandBase {
  private Drivetrain drivetrain;
  private double speed;
  private double ticksNeeded;
  private double meters;
  private double startEncoderDist;

  /** Creates a new DriveForwardMeters. */
  public DriveForwardMeters(Drivetrain drivetrain, double speed, double meters) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.meters = meters;
    startEncoderDist = drivetrain.getRightEncoderDistanceMeters();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticksNeeded = meters*MetersToTicks;
    drivetrain.driveLR(speed, speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (meters<=((startEncoderDist*TicksToMeeters)-drivetrain.getRightEncoderDistanceMeters()));
  }
}
