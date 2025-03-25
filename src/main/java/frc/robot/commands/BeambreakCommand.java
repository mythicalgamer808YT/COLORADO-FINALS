package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BeamBreakSubsystem;

public class BeambreakCommand extends Command {
    private final BeamBreakSubsystem beambreak;
    private final SequentialCommandGroup commandToRun;

    public BeambreakCommand(BeamBreakSubsystem beambreakSubsystem, SequentialCommandGroup commandToRun) {
        this.beambreak = beambreakSubsystem;
        this.commandToRun = commandToRun;
    }

    
}
