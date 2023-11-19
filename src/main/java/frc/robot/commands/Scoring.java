package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ScoringThing;
import edu.wpi.first.wpilibj.PS4Controller;

public class Scoring extends CommandBase{
    private final ScoringThing m_scoringMotor; 
    private final PS4Controller m_scoringtrigger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ScoringThing The subsystem used by this command.
   */    
    public Scoring(ScoringThing scoring, PS4Controller scoringController) {
        m_scoringMotor = scoring;
        m_scoringtrigger = scoringController;

        addRequirements(scoring);

    }

    @Override
    public void initialize() {

    }

    @Override
  public void execute() {
    m_scoringMotor.runScoring(m_scoringtrigger.getRightY());
}

@Override
public void end(boolean interrupted) {

}
@Override
public boolean isFinished() {
  return false;
}
}