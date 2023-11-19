package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ScoringThing extends SubsystemBase {
  
    private final WPI_TalonSRX m_scoringMotor;
    public ScoringThing() {
    m_scoringMotor = new WPI_TalonSRX(4);
    m_scoringMotor.setNeutralMode(NeutralMode.Brake);

    }


public void runScoring(double speed) {
    m_scoringMotor.set(speed);
}
}

