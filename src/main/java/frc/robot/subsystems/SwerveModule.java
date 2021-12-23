package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Tuple;

import static frc.robot.Constants.*;

import java.util.*;

import static frc.robot.Constants.*;

public class SwerveModule {

    public enum Position {
        FrontRight,
        FrontLeft,
        BackRight,
        BackLeft
    }

    //Each motor has a different angle when rotated, so keep track of that here.
    private final Map<Position, Tuple<Double, Double>> rotationMap = new HashMap<>() {{
        put(Position.FrontRight, new Tuple<>(1., -1.));
        put(Position.FrontLeft, new Tuple<>(1., 1.));
        put(Position.BackRight, new Tuple<>(-1., -1.));
        put(Position.BackLeft, new Tuple<>(-1., 1.));
    }};

    private final int canSpin;
    private final int canDrive;
    protected boolean invertSpin;
    protected boolean invertDrive;
    protected final Position position;
    protected final int offset;

    protected final TalonSRX talon;
    protected final CANSparkMax spark;

    public SwerveModule(int canSpin, int canDrive, int offset, Position position, boolean invertSpin, boolean invertDrive) {
        this.canSpin = canSpin;
        this.canDrive = canDrive;
        this.position = position;
        this.offset = offset;
        this.invertSpin = invertSpin;
        this.invertDrive = invertDrive;
        this.talon = new TalonSRX(canSpin);
        this.spark = new CANSparkMax(canDrive, CANSparkMaxLowLevel.MotorType.kBrushless);

        configure();
    }

    public SwerveModule(int canSpin, int canDrive, int offset, Position position) {
        this(canSpin, canDrive, offset, position, false, false);
    }

    private void configure(){
        this.talon.configFactoryDefault();
        this.spark.restoreFactoryDefaults();
        this.talon.configFactoryDefault(TIMEOUT_MS);

        this.talon.setSensorPhase(invertSpin);
        this.talon.setInverted(invertSpin);

        this.spark.setInverted(invertDrive);

        this.talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, PID_IDX, TIMEOUT_MS);
        this.talon.configAllowableClosedloopError(PID_IDX, ALLOWABLE_ERROR, TIMEOUT_MS);

        this.talon.config_kF(PID_IDX, 0.0, TIMEOUT_MS);
        this.talon.config_kP(PID_IDX, DRIVE_P, TIMEOUT_MS);
        this.talon.config_kI(PID_IDX, DRIVE_I, TIMEOUT_MS);
        this.talon.config_kD(PID_IDX, DRIVE_D, TIMEOUT_MS);

        this.talon.configNominalOutputForward(0, TIMEOUT_MS);
        this.talon.configNominalOutputReverse(0, TIMEOUT_MS);
        this.talon.configPeakOutputForward(PEAK_OUTPUT, TIMEOUT_MS);
        this.talon.configPeakOutputReverse(-PEAK_OUTPUT, TIMEOUT_MS);
    }

    public void zeroHeading(){
        this.talon.set(ControlMode.Position, this.offset);
    }

    //https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/lib/util/CTREModuleState.java
    public void setHeading(double degrees){
        double targetAngle = degrees;
        double delta = targetAngle - getCurrentAngle();
        if (Math.abs(delta) > 90){
            this.setInvertDrive(true);
            targetAngle = delta > 90 ? (targetAngle - 180.) : (targetAngle + 180.);
        }
        else {
            this.setInvertDrive(false);
        }

        double targetClicks = ((targetAngle / 360.) * SPIN_NUM_CLICKS) + this.offset;

        this.talon.set(ControlMode.Position, targetClicks);
    }

    public void setInvertDrive(boolean inverted){
        //XOR
        this.spark.setInverted(invertDrive ^ inverted);
    }

    public double getCurrentAngle(){
        return ((this.talon.getSelectedSensorPosition(PID_IDX) - this.offset) / SPIN_NUM_CLICKS) * 360;
    }

    public void setSpeed(double speed){
        this.spark.set(speed);
    }

    public void printStats(){
        System.out.println("------Start------");
        System.out.println("\t: " + this);
        System.out.println("------End------");
    }

    @Override
    public String toString() {
        return "SwerveModule{" +
                "position=" + position +
                ", canSpin=" + canSpin +
                ", canDrive=" + canDrive +
                ", invertSpin=" + invertSpin +
                ", invertDrive=" + invertDrive +
                ", offset=" + offset +
                ", currentClicks=" + this.talon.getSelectedSensorPosition() +
                '}';
    }

    public boolean isSpinning(){
        return this.talon.getClosedLoopError() > ALLOWABLE_ERROR;
    }

    public Tuple<Double, Double> getRotationalSpeeds(double zMag){
        Tuple<Double, Double> rm = rotationMap.get(this.position);
        return new Tuple<>(rm.x * zMag, rm.y * zMag);
    }
}
