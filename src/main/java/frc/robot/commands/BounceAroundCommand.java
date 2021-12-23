package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

public class BounceAroundCommand extends CommandBase {

    private final SwerveSubsystem subsystem;
    private final List<Double> positions = List.of(0., 180., 270., 90., 0., 45., 225., 135., 315.); //List.of(0., 10., 350., 170., 180.);
    private int index = 0;
    private long waitTime = 2500;
    private long lastWait = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public BounceAroundCommand(SwerveSubsystem subsystem)
    {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.subsystem.initialize();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.subsystem.modules(sm -> sm.setSpeed(.1));

        if(lastWait + waitTime < System.currentTimeMillis()){
            lastWait = System.currentTimeMillis();
            index = (index + 1) % positions.size();
            System.out.println(positions.get(index));
            this.subsystem.modules(sm -> sm.setHeading(positions.get(index)));
        }
    }


}
