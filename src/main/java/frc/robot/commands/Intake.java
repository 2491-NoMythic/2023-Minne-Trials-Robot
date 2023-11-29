package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeThing;
import edu.wpi.first.wpilibj.PS4Controller;

public class Intake extends CommandBase {
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
        if(m_intakeTrigger.getR1Button()) {
            m_intakeMotor.runIntake(-1);
        } else {
        if(m_intakeTrigger.getR2Button()) {
            m_intakeMotor.runIntake(-0.2);
        } else {
        if(m_intakeTrigger.getL2Button() || m_intakeTrigger.getL2Button()) {
            m_intakeMotor.runIntake(0);
        } else {
        if(m_intakeTrigger.getL1Button()) {
            m_intakeMotor.runIntake(.3);
        ;  
        }}}}
        double throttle = m_intakeTrigger.getLeftY();
       if (Math.abs(throttle) < Constants.Dead)
                throttle = 0;
        else 
            throttle = Math.signum(throttle) * (Math.abs(throttle) - Constants.Dead) / (1-Constants.Dead);
        m_intakeMotor.runIntake(throttle);
    }
    @Override
    public void end(boolean interrupted) {
        m_intakeMotor.runIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
