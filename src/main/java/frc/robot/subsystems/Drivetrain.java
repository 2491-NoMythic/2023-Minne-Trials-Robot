// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.Time;
import java.util.Random;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.pathplanner.lib.PathPlannerTrajectory;
import static frc.robot.Constants.DriveTrainConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;



public class Drivetrain extends SubsystemBase {
    /** Creates a new Drivetrain. */
    private final WPI_TalonFX m_leftDrive;
    private final WPI_TalonFX m_rightDrive;
    private final DifferentialDrive m_robotDrive;
    private final DifferentialDriveOdometry m_Odometry;

    TalonSRX PigeonController = new TalonSRX(Constants.pigoenid);
    private final PigeonIMU Gyro = new PigeonIMU(PigeonController);
    public DifferentialDriveKinematics kinematics = DriveTrainConstants.kinematics;
    private final Field2d m_field = new Field2d();



    public Drivetrain() {
        m_leftDrive = new WPI_TalonFX(1);
        m_rightDrive = new WPI_TalonFX(2);
        m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
        m_rightDrive.setNeutralMode(NeutralMode.Brake);
        m_leftDrive.setNeutralMode(NeutralMode.Brake);

        m_leftDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_rightDrive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        //FIX FIX FIX
        m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters(), new Pose2d());
        // m_Odometry = new m_Odometry();
        Gyro.setYaw(0);
        SmartDashboard.putData("Field", m_field);

        resetOdometry(getPose());

       
    }

    public Pose2d getPose() {
        return m_Odometry.getPoseMeters();
    }

    public void resetEncoders()
    {
        m_rightDrive.setSelectedSensorPosition(0);
        m_leftDrive.setSelectedSensorPosition(0);
    }



   public void resetOdometry(Pose2d pose) {
        resetEncoders();
       m_Odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters(),
                pose);
    }


	public void updateOdometry() {
		// odometry.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation());
		m_Odometry.update(Rotation2d.fromDegrees(getYaw()), getLeftEncoderDistanceMeters() , getRightEncoderDistanceMeters() * -1);
	}

    public double getLeftEncoderDistanceMeters()
    {
     // System.out.print(m_leftDrive.getSelectedSensorPosition());
      //return()
      return (m_leftDrive.getSelectedSensorPosition() * Constants.TicksToMeeters);
    }

    public double getRightEncoderDistanceMeters()
    {
      return (m_rightDrive.getSelectedSensorPosition() * Constants.TicksToMeeters);
    }

    public void displayFieldTrajectory(PathPlannerTrajectory traj) {
        m_field.getObject("traj").setTrajectory(traj);
    }

    public double getYaw() {
        return Gyro.getYaw() % 360;
    }

    public void drive(double xSpeed, double zRotation) {
        m_robotDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void driveLR(double lspeed, double rspeed) {
        m_robotDrive.tankDrive(-lspeed, rspeed);
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("left power", lspeed);
        SmartDashboard.putNumber("right power", rspeed);

    }

    public double getHeading() {
        return m_Odometry.getPoseMeters().getRotation().getDegrees();
    }

    public void stop() {
        m_robotDrive.tankDrive(0, 0);
    }

    @Override
    public void periodic() {
      updateOdometry();
        SmartDashboard.putNumber("get Yaw", getYaw());
        m_field.setRobotPose(m_Odometry.getPoseMeters());
       
        SmartDashboard.putNumber("Left encoder", getLeftEncoderDistanceMeters());
        SmartDashboard.putNumber("Right encoder", getRightEncoderDistanceMeters());
        SmartDashboard.putNumber("X", m_Odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", m_Odometry.getPoseMeters().getY());
       
        // This method will be called once per scheduler run
    }
}


