// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.*;
import java.util.function.Consumer;

import static frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase
{
    private List<SwerveModule> modules;
    private SwerveDriveKinematics kinematics;

    //Probably doesn't need to be in this subsystem
    private final AHRS gyro;

    /** Creates a new ExampleSubsystem. */
    public SwerveSubsystem()
    {
        gyro = new AHRS(SerialPort.Port.kMXP);

        modules = new LinkedList<>();
        double loc = Constants.SWERVE_LENGTH/2.0;

        //FrontRight
        modules.add(new SwerveModule(A_SPIN_ID, A_DRIVE_ID, A_OFFSET, false, true, loc, loc));
        //FrontLeft
        modules.add(new SwerveModule(B_SPIN_ID, B_DRIVE_ID, B_OFFSET, true, true, loc, -loc));
        //BackLeft
        modules.add(new SwerveModule(C_SPIN_ID, C_DRIVE_ID, C_OFFSET, false, true, -loc, -loc));
        //BackRight
        modules.add(new SwerveModule(D_SPIN_ID, D_DRIVE_ID, D_OFFSET, true, true, -loc, loc));

        // The order of this list mirrors the order in setStates
        kinematics = new SwerveDriveKinematics(modules.stream().map(m -> m.getTranslation()).toArray(Translation2d[]::new));
    }

    public void initialize(){
        resetDistances();
        modules(m -> m.zeroHeading());
        this.gyro.reset();
    }

    public void resetDistances(){
        modules(m -> m.resetDistance());
    }

    @Override
    public void periodic()
    {
        modules(m -> m.periodic());
    }

    public void setHeadings(double angle){
        modules(m -> m.setHeading(angle));
    }

    public void setVelocities(double velo){
        modules(m -> m.setVelocity(velo));
    }

    public void modules(Consumer<SwerveModule> action){
        modules.forEach(action);
    }

    public double getYaw(){
        return gyro.getYaw();
    }

    public void resetGyro(){
        this.gyro.reset();
    }

    public SwerveModuleState[] getStates(){
        return modules.stream().map(m -> m.getState()).toArray(SwerveModuleState[]::new);
    }

    // The order of this list mirrors the order in the constructor
    public void setStates(SwerveModuleState[] states){
        for(int i = 0; i < states.length; i++){
            modules.get(i).setState(states[i]);
        }
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldRelative) {
        double maxVelo = 3;
        double x = clip(xSpeed, .1) * maxVelo;
        double y = clip(ySpeed, .1) * maxVelo;
        double z = clip(zSpeed, .1) * maxVelo;
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(-x, y, z, this.gyro.getRotation2d()) : new ChassisSpeeds(-x, y, z));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, maxVelo);
        setStates(swerveModuleStates);
    }

    private static double clip(double val, double clip){
        if(Math.abs(val) < clip){
            return 0;
        }
        else {
            return val;
        }
    }

}
