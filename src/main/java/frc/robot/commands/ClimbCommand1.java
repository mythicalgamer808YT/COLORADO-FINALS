package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.ClimbStates;
import frc.robot.subsystems.ClimbSubsystem1;

public class ClimbCommand1 extends Command {
    private final ClimbSubsystem1 climbSubsystem1;
    private final ClimbStates climbState1;
    public ClimbCommand1 (ClimbSubsystem1 climbSubsystem1, ClimbStates climbState1) {
        this.climbSubsystem1 = climbSubsystem1;
        this.climbState1 = climbState1;
    }

    @Override
    public void initialize () {
    climbSubsystem1.setClimbSpeed (climbState1.speed);
    }
}
