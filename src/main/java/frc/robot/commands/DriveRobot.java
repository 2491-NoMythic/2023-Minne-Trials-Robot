package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class DriveRobot extends CommandBase {
  private final Drivetrain m_robotDrive;
  private final Joystick m_robotsJoystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param Drivetrain The subsystem used by this command.
   */
  public DriveRobot(Drivetrain robotDrives, Joystick robJoystick) {
    m_robotDrive = robotDrives;
    m_robotsJoystick = robJoystick;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrives);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = m_robotsJoystick.getThrottle();
m_robotDrive.drive(m_robotsJoystick.getZ() * 0.80, m_robotsJoystick.getY() * (throttle +2));
SmartDashboard.putNumber("Throttle Value", m_robotsJoystick.getThrottle());
SmartDashboard.putNumber("Joystick Input speed", m_robotsJoystick.getX());
SmartDashboard.putNumber("Joystick Input Rotation", m_robotsJoystick.getZ());
  }
 
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}