package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.EscalatorConstants;

public class Escalator extends SubsystemBase{
    private final CANSparkMax spark1;
    public Escalator(){
        spark1 = new CANSparkMax(EscalatorConstants.escalatorMotor, MotorType.kBrushless);
    }
    public void runThing(double Speed){
        spark1.set(Speed);
    }
}
