package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter extends Command implements Subsystem {

    private boolean hasShot;
    public Shooter(){

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
        return hasShot;
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Command was interrupted.");
        }
    }
}

