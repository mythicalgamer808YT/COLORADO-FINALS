package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.InnerElevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.PrimaryElevator;

public class ElevatorCommandHandler {
    private final PrimaryElevator primaryElevatorSubsystem;
    private final InnerElevator innerElevatorSubsystem;
    private final IntakeArm armSubsystem;

    public ElevatorCommandHandler(PrimaryElevator primaryElevatorSubsystem, InnerElevator innerElevatorSubsystem, IntakeArm armSubsystem) {
        this.primaryElevatorSubsystem = primaryElevatorSubsystem;
        this.innerElevatorSubsystem = innerElevatorSubsystem;
        this.armSubsystem = armSubsystem;

        primaryElevatorSubsystem.setPrimaryElevatorState(ElevatorStates.STARTING_POSITION);
        innerElevatorSubsystem.setInnerElevatorState(ElevatorStates.STARTING_POSITION);
        armSubsystem.setArmState(ElevatorStates.STARTING_POSITION);
    }

    public InstantCommand setPrimaryElevatorState(ElevatorStates state) {
        return new InstantCommand(() -> primaryElevatorSubsystem.setPrimaryElevatorState(state), primaryElevatorSubsystem);
    }

    public InstantCommand setInnerElevatorState(ElevatorStates state) {
        return new InstantCommand(() -> innerElevatorSubsystem.setInnerElevatorState(state), innerElevatorSubsystem);
    }

    public InstantCommand setArmState(ElevatorStates state) {
        return new InstantCommand(() -> armSubsystem.setArmState(state), armSubsystem);
    }

    public SequentialCommandGroup setElevators(ElevatorStates state) {
        if((innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.L1) || innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.L2))
            && (state.equals(ElevatorStates.PRE_INTAKE) || state.equals(ElevatorStates.INTAKE) || state.equals(ElevatorStates.STARTING_POSITION))) {
            return new SequentialCommandGroup(setPrimaryElevatorState(state), setInnerElevatorState(ElevatorStates.PRE_INTAKE), new WaitCommand(0.3), 
                setArmState(state), new WaitCommand(0.4), setInnerElevatorState(state));
        } else if(innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.PRE_INTAKE) && state.equals(ElevatorStates.INTAKE)) {
            return new SequentialCommandGroup(setPrimaryElevatorState(state), setArmState(state), new WaitCommand(0.4), setInnerElevatorState(state));
        } else {
            return new SequentialCommandGroup(setPrimaryElevatorState(state), setInnerElevatorState(state), setArmState(state));
        }
    }

    public ElevatorStates getState() {
        return innerElevatorSubsystem.getInnerElevatorState();
    }
}
//  