package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkSystem extends SubsystemBase{
    private final CANSparkMax spark1;
    private final CANSparkMax spark2;
    public SparkSystem(){
        spark1 = new CANSparkMax(1, MotorType.kBrushless);
        spark2 = new CANSparkMax(2, MotorType.kBrushless);
    }
    public void runThing(double Speed){
        spark1.set(Speed);
        spark2.set(-Speed);
    }
}
