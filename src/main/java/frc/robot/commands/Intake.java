package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeThing;
import edu.wpi.first.wpilibj.PS4Controller;

public class Intake extends CommandBase{
    private final IntakeThing m_intakeMotor; 
    private final PS4Controller m_intakeTrigger;

  /**
   * Creates a new ExampleCommand.
   *
   * @param IntakeThing The subsystem used by this command.
   */    
    public Intake(IntakeThing intake, PS4Controller intakController) {
        m_intakeMotor = intake;
        m_intakeTrigger = intakController;

        addRequirements(intake);

    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intakeMotor.runIntake(m_intakeTrigger.getLeftY());
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}

