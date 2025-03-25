package frc.robot.commands.teleopCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.InnerElevator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.Constants.ControllerConsts;
import frc.robot.States.ElevatorStates;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;


public class teleopCommand extends Command {
    CommandSwerveDrivetrain drivetrain;
    ControllerSubsystem controllerSubsystem;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1);
    InnerElevator i; 

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    double strafeSpeed;

    public teleopCommand(ControllerSubsystem c, CommandSwerveDrivetrain d, InnerElevator i) {
        this.controllerSubsystem = c;
        this.drivetrain = d;
        this.i = i; 

        this.strafeSpeed = MaxSpeed * ControllerConsts.STRAFE_RATIO;
        addRequirements(drivetrain);
    }

    @Override 
    public void execute() {
        if(!controllerSubsystem.getPrecision()) {
            if(!controllerSubsystem.getStrafe()) {
                if(i.getInnerElevatorState().equals(ElevatorStates.STARTING_POSITION)) {
                    controllerSubsystem.setElevatorUp(false);
                } else {
                    controllerSubsystem.setElevatorUp(true);
                }

                // System.out.println("DRIVING");
                drivetrain.applyRequest(() ->
                drive.withVelocityX(-Utilities.polynomialAccleration(controllerSubsystem.getPosY() * MaxSpeed * 0.5)) // Drive forward with negative Y (forward)
                    .withVelocityY(-Utilities.polynomialAccleration(controllerSubsystem.getPosX() * MaxSpeed * 0.5)) // Drive left with negative X (left)
                    .withRotationalRate(-Utilities.polynomialAccleration(controllerSubsystem.getRightPosX()) * MaxAngularRate)// Drive counterclockwise with negative X (left)
                ).execute(); 
            } else {
                //strafe Drive
                if(controllerSubsystem.getStrafeLeft()) {
                    //left
                    strafeSpeed *= -1;
                } 
                
                //idk if correct input ofc.
                drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
                    .withVelocityY(strafeSpeed).withRotationalRate(0)).execute();
            }
        } else {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Utilities.polynomialAccleration(controllerSubsystem.getPosY() * MaxSpeed * ControllerConsts.PRECIECE_ADD_TRANSLATIONAL_DEADBAND_RATIO)) // Drive forward with negative Y (forward)
                    .withVelocityY(-Utilities.polynomialAccleration(controllerSubsystem.getPosX() * MaxSpeed * ControllerConsts.PRECIECE_ADD_TRANSLATIONAL_DEADBAND_RATIO)) // Drive left with negative X (left)
                    .withRotationalRate(-Utilities.polynomialAccleration(controllerSubsystem.getRightPosX()) * MaxAngularRate * ControllerConsts.PRECIECE_ADD_ROTATIONAL_DEADBAND_RATIO)// Drive counterclockwise with negative X (left)
            ).execute();
        }
    }
}