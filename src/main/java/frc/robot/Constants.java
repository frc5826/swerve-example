// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be declared
 * globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static final int A_SPIN_ID = 2;
    public static final int A_DRIVE_ID = 3;
    public static final int A_OFFSET = 1369;

    public static final int B_DRIVE_ID = 4;
    public static final int B_SPIN_ID = 5;
    public static final int B_OFFSET = 594;

    public static final int C_SPIN_ID = 6;
    public static final int C_DRIVE_ID = 7;
    public static final int C_OFFSET = 300;

    public static final int D_DRIVE_ID = 8;
    public static final int D_SPIN_ID = 9;
    public static final int D_OFFSET = 2357;

    public static final double SPIN_NUM_CLICKS = 4096;

    public static final Joystick joystick = new Joystick(0);

    public static final int PID_IDX = 0;
    public static final int TIMEOUT_MS = 5000;
    public static final double PEAK_OUTPUT = 1.0;
    public static final double ALLOWABLE_ERROR = 10;

    public static final double DRIVE_P = 2.0;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.5;

}
