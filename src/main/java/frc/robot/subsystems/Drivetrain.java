// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX m_leftDrive;
  private final WPI_TalonFX m_rightDrive;
  private final DifferentialDrive m_robotDrive;

  TalonSRX PigeonController = new TalonSRX(Constants.pigoenid);
  private final PigeonIMU Gyro = new PigeonIMU (PigeonController);

public Drivetrain() {
    m_leftDrive = new WPI_TalonFX(1);
    m_rightDrive = new WPI_TalonFX(2);
    m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
    m_rightDrive.setNeutralMode(NeutralMode.Brake);
    m_leftDrive.setNeutralMode(NeutralMode.Brake); 
    
  } 
public double getYaw()
{
 return Gyro.getYaw();
}



    
  public void drive (double xSpeed, double zRotation) {
    m_robotDrive.arcadeDrive(xSpeed, zRotation);
  }
  @Override
  public void periodic() {
    System.out.println(getYaw());
    // This method will be called once per scheduler run
  }
}
