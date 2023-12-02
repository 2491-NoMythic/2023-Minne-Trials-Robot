package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeThing extends SubsystemBase {
    private final WPI_TalonSRX m_intakeMotor;

    public IntakeThing() {
        m_intakeMotor = new WPI_TalonSRX(3);
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }
    
    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public void runAtDefault() {
        m_intakeMotor.set(0.8);
    }

}
