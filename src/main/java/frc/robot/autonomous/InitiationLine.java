package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class InitiationLine extends CommandBase{
    public InitiationLine(){
        
        addRequirements(Robot.drivetrain);
        
    }

    @Override
    public void initialize(){
        Robot.drivetrain.driveDistanceMagic(6); //TODO how many feet to go?
    }

}