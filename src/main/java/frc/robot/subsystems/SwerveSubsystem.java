// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Tuple;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import static frc.robot.subsystems.SwerveModule.Position.*;
import static frc.robot.Constants.*;

public class SwerveSubsystem extends SubsystemBase
{

    private final SwerveModule a;
    private final SwerveModule b;
    private final SwerveModule c;
    private final SwerveModule d;
    private Map<SwerveModule.Position, SwerveModule> modules;

    /** Creates a new ExampleSubsystem. */
    public SwerveSubsystem()
    {
        modules = new HashMap<>();

        a = new SwerveModule(A_SPIN_ID, A_DRIVE_ID, A_OFFSET, FrontRight, false, true);
        modules.put(FrontRight, a);

        b = new SwerveModule(B_SPIN_ID, B_DRIVE_ID, B_OFFSET, FrontLeft, true, true);
        modules.put(FrontLeft, b);

        c = new SwerveModule(C_SPIN_ID, C_DRIVE_ID, C_OFFSET, BackLeft, false, true);
        modules.put(BackLeft, c);

        d = new SwerveModule(D_SPIN_ID, D_DRIVE_ID, D_OFFSET, BackRight, true, true);
        modules.put(BackRight, d);
    }

    public void initialize(){
        modules(sm -> sm.zeroHeading());
    }

    @Override
    public void periodic()
    {

    }


    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    // y -> forward/reverse
    // x -> side to side
    // z -> rotation
    public void drive(double y, double x, double z){
        // The joystick is noisy and causes a lot of extra wheel spin
        double cy = clip(y, .1);
        double cx = clip(x, .1);
        double cz = clip(z, .1);

        double zMag = Math.signum(cz) * Math.sqrt(Math.pow(cz, 2) / 2.);

        //If any of the magnitudes are greater than 1 we want to normalize them
        double tmpmax = 1.;
        Map<SwerveModule.Position, Tuple<Double, Double>> mna = new HashMap<>();
        for(SwerveModule sm : modules()){
            Tuple<Double, Double> rotation = sm.getRotationalSpeeds(zMag);
            double nx = cx + rotation.x;
            double ny = cy + rotation.y;
            double angle = Math.toDegrees(Math.atan2(nx, ny));
            double mag = Math.sqrt(Math.pow(nx, 2) + Math.pow(ny, 2));
            if(mag > tmpmax){
                tmpmax = mag;
            }
            mna.put(sm.position, new Tuple<>(mag, angle));
        }

        System.out.println(mna);

        final double max = tmpmax;
        this.modules(sm -> sm.setHeading(mna.get(sm.position).y));
        this.modules(sm -> sm.setSpeed(mna.get(sm.position).x / max));

    }

    private static double clip(double val, double clip){
        if(Math.abs(val) < clip){
            return 0;
        }
        else {
            return val;
        }
    }

    public void modules(Consumer<SwerveModule> action){
        modules.values().forEach(action);
    }

    public List<SwerveModule> modules(){
        return new LinkedList<>(modules.values());
    }

    public SwerveModule getModule(SwerveModule.Position position){
        return this.modules.get(position);
    }
}
