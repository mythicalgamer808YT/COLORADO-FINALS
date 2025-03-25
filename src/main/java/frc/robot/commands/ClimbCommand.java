package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.States.ClimbStates;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand {
    private final ClimbSubsystem climbSubsystem;
    
    private ClimbStates state; 
    
    public ClimbCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
    }

    public ClimbStates getCurrentState() {
        return state;
    }

    public InstantCommand setClimbState(ClimbStates state) {
        this.state = state;
        return new InstantCommand(() -> climbSubsystem.setClimbState(state), climbSubsystem);
    }
}
