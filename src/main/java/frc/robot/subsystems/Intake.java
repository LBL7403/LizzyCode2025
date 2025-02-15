package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake extends Command implements Subsystem {

    private boolean hasMoved;
    public Intake(){

    }

    @Override
    public void initialize() {
        System.out.println("Command initialized!");
    }
    
    @Override
    public void execute() {
        
    }
    
    @Override
    public boolean isFinished() {
        return hasMoved;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Command was interrupted.");
        }
    }
}

