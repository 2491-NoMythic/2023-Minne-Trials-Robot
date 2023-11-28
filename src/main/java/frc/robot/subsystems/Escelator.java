package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Escelator extends SubsystemBase{
    private final CANSparkMax spark1;
    public Escelator(){
        spark1 = new CANSparkMax(1, MotorType.kBrushless);
    }
    public void runThing(double Speed){
        spark1.set(Speed);
    }
}
