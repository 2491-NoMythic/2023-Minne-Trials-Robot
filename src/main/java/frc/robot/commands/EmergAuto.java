package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
public class EmergAuto extends CommandBase{
    private double autonomousStartTime;
    private Drivetrain m_Drivetrain;
    public EmergAuto(Drivetrain drivetrain){
        m_Drivetrain = drivetrain;
    }
    @Override
    public void initialize() {
        autonomousStartTime = Timer.getFPGATimestamp();
    }
    @Override
    public void execute() {
        // Drive forward for 3 seconds (adjust the time as needed)
        if (Timer.getFPGATimestamp() - autonomousStartTime < 2.0) {
        m_Drivetrain.driveLR(0.5, 0.5);   
        } else {
            // Stop the motors after 3 seconds
        m_Drivetrain.stop();
        }
    }
}
