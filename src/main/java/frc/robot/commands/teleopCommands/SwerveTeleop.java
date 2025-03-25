package frc.robot.commands.teleopCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Utilities;
import frc.robot.Constants;
import frc.robot.States.ElevatorStates;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevator;

// ensure smooth acceleration with variable acceleration in different states 
public class SwerveTeleop extends Command {
    private final CommandSwerveDrivetrain drivetrain; 
    private final InnerElevator innerElevator;
    private final CommandXboxController controller;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SlewRateLimiter rLimiter; 

    private final SlewRateLimiter xLimiterMax; 
    private final SlewRateLimiter yLimiterMax;

    private final SlewRateLimiter xLimiterRestricted; 
    private final SlewRateLimiter yLimiterRestricted;

    private ElevatorStates previousState; 
    private ElevatorStates currentState;

    private double xSpeed, ySpeed, rSpeed;
    private double xInput, yInput, rInput; 

    public SwerveTeleop(CommandSwerveDrivetrain drivetrain, InnerElevator innerElevator, CommandXboxController controller) {
        this.drivetrain = drivetrain; 
        this.innerElevator = innerElevator;
        this.controller = controller;

        rLimiter = new SlewRateLimiter(Constants.SLEW_ROTATION);

        xLimiterMax = new SlewRateLimiter(Constants.SLEW_POSITIVE_MAX, -Constants.SLEW_NEGATIVE_MAX, 0);
        yLimiterMax = new SlewRateLimiter(Constants.SLEW_POSITIVE_MAX, -Constants.SLEW_NEGATIVE_MAX, 0);

        xLimiterRestricted = new SlewRateLimiter(Constants.SLEW_POSITIVE_MIN, -Constants.SLEW_NEGATIVE_MIN, 0);
        yLimiterRestricted = new SlewRateLimiter(Constants.SLEW_POSITIVE_MIN, -Constants.SLEW_NEGATIVE_MIN, 0);

        xSpeed = 0; 
        ySpeed = 0; 
        rSpeed = 0; 

        xInput = 0; 
        yInput = 0; 
        rInput = 0; 

        currentState = ElevatorStates.STARTING_POSITION;
        previousState = ElevatorStates.STARTING_POSITION;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // accerlate slower if elevator is up 
        xInput = controller.getLeftY();
        yInput = controller.getLeftX();
        rInput = controller.getRightX();

        currentState = innerElevator.getInnerElevatorState(); 

        // if moving elevator states reset slew rate limiters
        // because of this could remove unused calculation for both since both limiters are kept consistant with this logic 
        // if(!currentState.equals(previousState)) {
        //     xLimiterMax.reset(rInput);
        //     yLimiterMax.reset(rInput);

        //     // keep both limiters updated
        //     xLimiterRestricted.reset(rInput);
        //     yLimiterRestricted.reset(rInput);
        // }
        
        // accelerate slower if inner elevator is up

    
    // if(innerElevator.getInnerElevatorState().equals(ElevatorStates.STARTING_POSITION)) {
            xSpeed = xLimiterMax.calculate(xInput);
            ySpeed = yLimiterMax.calculate(yInput);
            System.out.println(xInput);

            // keep both limiters updated
            xLimiterRestricted.calculate(xInput);
            yLimiterRestricted.calculate(yInput);
        // } else {
        //     xSpeed = xLimiterRestricted.calculate(xInput);
        //     ySpeed = yLimiterRestricted.calculate(yInput);

        //     // keep both limiters updated
        //     xLimiterMax.calculate(xInput);
        //     yLimiterMax.calculate(yInput);
        // }
        
        rSpeed = rLimiter.calculate(rInput);

        // clamp drive values to ensure that none go over -1 or 1 
        xSpeed = Utilities.clampDriveValues(xSpeed);
        ySpeed = Utilities.clampDriveValues(ySpeed);
        rSpeed = Utilities.clampDriveValues(rSpeed);
    
        // adjust to swerve max speed 
        // xSpeed *= MaxSpeed;
        // ySpeed *= MaxSpeed;
        // rSpeed *= MaxAngularRate;

        // // apply drive speed 
        // if(innerElevator.getPrecise()) {
        //     drivetrain.applyRequest(() -> drive.withVelocityX(-xSpeed * MaxSpeed * 0.5)
        //         .withVelocityY(-ySpeed * MaxSpeed * 0.5)
        //         .withRotationalRate(-rSpeed * MaxAngularRate * 0.8)).execute();
        // } else {
        drivetrain.applyRequest(() -> drive.withVelocityX(-xSpeed * MaxSpeed)
            .withVelocityY(-ySpeed * MaxSpeed)
            .withRotationalRate(-rSpeed * MaxAngularRate)).execute();

        // }
        System.out.println("XSPEED " + xSpeed + " :: YSPEED " + "ySpeed");
        previousState = currentState; 
        SmartDashboard.putNumber("neoaigaemou", xSpeed);
    }

}
