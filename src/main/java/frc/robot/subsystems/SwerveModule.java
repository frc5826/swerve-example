package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import static frc.robot.Constants.*;

import java.time.Duration;
import java.time.Instant;


public class SwerveModule {

    private final int canSpin;
    private final int canDrive;
    protected boolean invertSpin;
    protected boolean invertDrive;
    protected int offset;

    protected final TalonSRX talon;
    protected final CANSparkMax spark;

    private Instant timestamp;
    private double distance;
    private double rotationOffset;

    private Translation2d translation;

    public SwerveModule(int canSpin, int canDrive, int offset, boolean invertSpin, boolean invertDrive, double xloc, double yloc) {
        this.canSpin = canSpin;
        this.canDrive = canDrive;
        this.offset = offset;
        this.invertSpin = invertSpin;
        this.invertDrive = invertDrive;
        this.talon = new TalonSRX(canSpin);
        this.spark = new CANSparkMax(canDrive, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.timestamp = Instant.now();
        this.distance = 0;
        this.rotationOffset = 0;
        this.translation = new Translation2d(xloc, yloc);

        configure();
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
        this.talon.config_kP(PID_IDX, STEER_P, TIMEOUT_MS);
        this.talon.config_kI(PID_IDX, STEER_I, TIMEOUT_MS);
        this.talon.config_kD(PID_IDX, STEER_D, TIMEOUT_MS);

        this.talon.configNominalOutputForward(0, TIMEOUT_MS);
        this.talon.configNominalOutputReverse(0, TIMEOUT_MS);
        this.talon.configPeakOutputForward(PEAK_OUTPUT, TIMEOUT_MS);
        this.talon.configPeakOutputReverse(-PEAK_OUTPUT, TIMEOUT_MS);

        this.spark.getPIDController().setP(DRIVE_P);
        this.spark.getPIDController().setI(DRIVE_I);
        this.spark.getPIDController().setD(DRIVE_D);
        this.spark.getPIDController().setOutputRange(-PEAK_OUTPUT, PEAK_OUTPUT);

        //It appears you can't invert a brushless motor's encoder, so we'll have to invert the velocity manually.
        //this.spark.getEncoder().setInverted(true);

        //https://github.com/FRCTeam2910/Common/blob/master/robot/src/main/java/org/frcteam2910/common/robot/drivers/Mk2SwerveModuleBuilder.java
        //Odometry is speedMetersPerSecond, the default for the encoder is RPM.
        //So we need to give the encoder a number to make RPM to MPS
        // ((1 rot * Pi * Diameter) / (60 * 5.25)
        this.spark.getEncoder().setVelocityConversionFactor((Math.PI * WHEEL_DIAMETER) / (60.0 * DRIVE_GEAR_RATIO));
    }

    public void zeroHeading(){
        this.talon.set(ControlMode.Position, this.offset);
    }

    //https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/lib/util/CTREModuleState.java
    public void setHeading(double degrees){
        double targetAngle = degrees;
        double delta = targetAngle - getAngle();
        if (Math.abs(delta) > 90){
            this.setInvertDrive(true);
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
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

    public double getAngle(){
        return ((this.talon.getSelectedSensorPosition(PID_IDX) - this.offset) / SPIN_NUM_CLICKS) * 360;
    }

    //Meters per second
    public double getVelocity(){
        return spark.getEncoder().getVelocity();
    }

    public void setVelocity(double mps){
        this.spark.getPIDController().setReference(mps, ControlType.kVelocity);
    }

    public void periodic()
    {
        Instant newTimestamp = Instant.now();
        double elapsed = Duration.between(newTimestamp, timestamp).toMillis();
        double velo = getVelocity() / 1000.0;
        distance += velo * elapsed;
        timestamp = newTimestamp;
    }

    public void resetDistance(){
        this.distance = 0;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(this.getVelocity(), Rotation2d.fromDegrees(this.getAngle()));
    }

    public void setState(SwerveModuleState state){
        this.setVelocity(state.speedMetersPerSecond);
        this.setHeading(state.angle.getDegrees());
    }

    public Translation2d getTranslation(){
        return this.translation;
    }
}
