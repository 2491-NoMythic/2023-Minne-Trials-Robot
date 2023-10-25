package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SparkSystem;

public class DoSparkSystem extends CommandBase{
    private final SparkSystem m_SparkSystem;
    public DoSparkSystem(SparkSystem sparkSystem){
        m_SparkSystem = sparkSystem;
        addRequirements(sparkSystem);
    }
     // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SparkSystem.runThing(1);
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
