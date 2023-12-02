// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int pigoenid = 3;

  public static final double TicksToMeeters = (1/2048)*(1/10.75)*(4*Math.PI)*(1/39.37);
  public static final double MetersToTicks = (1/TicksToMeeters);

  public static final boolean USE_AUTO_FROM_DASHBOARD = false;
  
  public static final double Dead = .3;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }
public static final class DriveTrainConstants {
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.66; //distance between the center of the wheelson each side
  public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DRIVETRAIN_TRACKWIDTH_METERS);
  public static final double MAX_POWER_OUTPUT = 0.3;
}
}
