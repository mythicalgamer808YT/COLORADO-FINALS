package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevatorSubsystem;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;


public class teleopCommand extends Command {
    CommandXboxController controller;
    CommandSwerveDrivetrain drivetrain;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1);
    InnerElevatorSubsystem i; 

    public teleopCommand(CommandXboxController c, CommandSwerveDrivetrain d, InnerElevatorSubsystem i) {
        controller = c;
        drivetrain = d;
        this.i = i; 
    }

    @Override 
    public void execute() {
        if(i.getInnerElevatorState().equals(ElevatorStates.STARTING_POSITION)) {
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-Utilities.polynomialAccleration(controller.getLeftX() * MaxSpeed)) // Drive forward with negative Y (forward)
                .withVelocityY(-Utilities.polynomialAccleration(controller.getLeftY() * MaxSpeed)) // Drive left with negative X (left)
                .withRotationalRate(-Utilities.polynomialAccleration(controller.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
        } else {
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-Utilities.polynomialAccleration(controller.getLeftY() * MaxSpeed * 0.2)) // Drive forward with negative Y (forward)
                .withVelocityY(-Utilities.polynomialAccleration(controller.getLeftX() * MaxSpeed * 0.2)) // Drive left with negative X (left)
                .withRotationalRate(-Utilities.polynomialAccleration(controller.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
        }
    }
}
