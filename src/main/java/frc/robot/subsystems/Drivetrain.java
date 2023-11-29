// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final DifferentialDriveOdometry m_Odometry;

    TalonSRX PigeonController = new TalonSRX(Constants.pigoenid);
    private final PigeonIMU Gyro = new PigeonIMU(PigeonController);


    public Drivetrain() {
        m_leftDrive = new WPI_TalonFX(1);
        m_rightDrive = new WPI_TalonFX(2);
        m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
        m_rightDrive.setNeutralMode(NeutralMode.Brake);
        m_leftDrive.setNeutralMode(NeutralMode.Brake);
        //FIX FIX FIX
        m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters(), new Pose2d());
        // m_Odometry = new m_Odometry();
        Gyro.setYaw(0);

    }

    public Pose2d getPose() {
        return m_Odometry.getPoseMeters();
    }

    public void resetEncoders()
    {

    }

    

   public void resetOdometry(Pose2d pose) {
        resetEncoders();
       m_Odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters(),
                pose);
    }


	public void updateOdometry() {
		// odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation());
		m_Odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
	}

    public double getLeftEncoderDistanceMeters()
    {
      return m_leftDrive.getSelectedSensorPosition(0)* Constants.TicksToMeeters;
    }

    public double getRightEncoderDistanceMeters()
    {
      return (m_rightDrive.getSelectedSensorPosition(0) * Constants.TicksToMeeters);
    }


    public double getYaw() {
        return Gyro.getYaw() % 360;
    }

    public void drive(double xSpeed, double zRotation) {
        m_robotDrive.arcadeDrive(xSpeed, zRotation);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("get Yaw", getYaw());
        updateOdometry();
        // This method will be called once per scheduler run
    }
}

