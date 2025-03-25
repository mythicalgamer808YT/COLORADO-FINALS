// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import java.time.zone.ZoneRulesException;
// import java.util.EmptyStackException;
// import java.util.function.Supplier;

// import java.util.NoSuchElementException;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.PhotonSubsystem;

// public class PhotonCommand1 extends Command{
//     private final PhotonSubsystem camera;
//     private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//     private final CommandSwerveDrivetrain drivetrain;
    
//     private final double goalYaw;
//     private final double goalX;
//     private final double goalY; 
//     private final Supplier<Double> yaw; 

//     private final PIDController xController;
//     private final PIDController yController;
//     private final PIDController theataController;

//     private Transform3d tagLocation; 

//     private double xSpeed;
//     private double ySpeed;
//     private double theataSpeed;

//     private int counter; 


//     public PhotonCommand1(PhotonSubsystem camera, CommandSwerveDrivetrain drive, double goalYaw, double goalX, double goalY, Supplier<Double> input) {
//         this.camera = camera;
//         this.drivetrain = drive;
        
//         xController = new PIDController(2, 0, 0);
//         yController = new PIDController(2, 0, 0);
//         theataController = new PIDController(0.1, 0,0 );
//         theataController.enableContinuousInput(-Math.PI, Math.PI);
//         this.yaw = input;
//                 this.goalX = goalX;
//         this.goalY = goalY;
//         this.goalYaw = goalYaw;
//         addRequirements(camera);
//     }

//     @Override
//     public void initialize() { 
//         xController.setSetpoint(goalX);
//         yController.setSetpoint(goalY);
//         theataController.setSetpoint(goalYaw);

//         xController.setTolerance(0.01);
//         yController.setTolerance(0.01);
//         theataController.setTolerance(0.01);
//     }

//     @Override
//     public void execute() {
//         tagLocation = camera.getTagLocation();

//         if(tagLocation != null) {
//             counter = 0; 
//             xSpeed = -xController.calculate(tagLocation.getX());
//             ySpeed = -yController.calculate(tagLocation.getY());
//             theataSpeed = -theataController.calculate(camera.getYaw());
//             //theataSpeed = theataController.calculate(camera.getYaw());
//         } else {
//             counter += 1;

//             xSpeed = 0;
//             ySpeed = 0;
//             theataSpeed = 0;
//         }
//         SmartDashboard.putNumber("x speed", xSpeed);
//         SmartDashboard.putNumber("Y sped", ySpeed);
//         SmartDashboard.putNumber("theat speed", theataSpeed);
//         drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(xSpeed)
//         .withVelocityY(ySpeed).withRotationalRate(theataSpeed)).execute();
//     }

//     @Override
//     public void end(boolean inFinished) {
//         drivetrain.applyRequest(() ->  robotCentricDrive.withVelocityX(0)
//         .withVelocityY(0).withRotationalRate(0)).execute();
//     }

//     @Override
//     public boolean isFinished() {
//         if(xController.atSetpoint() && yController.atSetpoint() && theataController.atSetpoint()) {
//             return true;
//         }
//         if(counter > 10) {
//             return true;
//         }
//         return false;
//     }
// }