package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.opencv.photo.AlignExposures;
import org.photonvision.proto.Photon;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.util.Utilities;
import frc.robot.Constants.PhotonConsts;
import frc.robot.States.ClimbStates;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;
import frc.robot.States.ReefPosition;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ElevatorCommandHandler;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.driveComand;
import frc.robot.commands.AprilTagCommands.AlignToReefTagRelative;
import frc.robot.commands.AprilTagCommands.OdometryCommand;
import frc.robot.commands.AprilTagCommands.OdometryCommand1;
import frc.robot.commands.AprilTagCommands.PhotonAlign;
import frc.robot.commands.AprilTagCommands.alignToPose;
import frc.robot.commands.teleopCommands.SwerveTeleop;
import frc.robot.commands.teleopCommands.basicTeleop;
import frc.robot.commands.teleopCommands.teleopCommand;
import frc.robot.subsystems.ClimbSubsystem1;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.InnerElevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.PrimaryElevator;
import frc.robot.subsystems.VisionVersions.Vision;
import frc.robot.subsystems.VisionVersions.Vision3;


//overall structure of the robot here, no real robot logic
//subsystems, commands, and triggermappings, respectively
public class RobotContainer {
    //generated swerve
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    //bindings and all for swerve control
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver0 = new CommandXboxController(0);
    private final CommandXboxController driver1 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private SendableChooser<Command> m_chooser;

    //** Subsystems **//
    private final PrimaryElevator s_primaryElevatorSubsystem = new PrimaryElevator();
    private final InnerElevator s_innerElevatorSubsystem = new InnerElevator();
    private final IntakeArm s_armSubsystem = new IntakeArm();
    private final ClimbSubsystem s_climbSubsystem = new ClimbSubsystem();
    private final BeamBreakSubsystem s_beambreakSubsystem = new BeamBreakSubsystem();
    private final ClimbSubsystem1 s_climbSubsystem1 = new ClimbSubsystem1();

    // private final Vision s_grayVision = new Vision("gray_photon_camera", drivetrain, PhotonConsts.GRAY_PHOTON_CAMERA_TO_ROBOT);
    // private final Vision s_blueVision = new Vision("blue_photon_camera", drivetrain, PhotonConsts.BLUE_PHOTON_CAMERA_TO_ROBOT);

    private final Vision3 s_grayVision1 = new Vision3("gray_photon_camera", drivetrain, PhotonConsts.GRAY_PHOTON_CAMERA_TO_ROBOT);
    private final Vision3 s_blueVision1 = new Vision3("blue_photon_camera", drivetrain, PhotonConsts.BLUE_PHOTON_CAMERA_TO_ROBOT);

    private final PhotonAlign leftPhotonAlign = new PhotonAlign(true, drivetrain, s_blueVision1, -0.17);
    private final PhotonAlign rightPhotonAlign = new PhotonAlign(false, drivetrain, s_grayVision1, 0.17);


    //** Command Handlers **//
    private final ElevatorCommandHandler ch_elevatorCommandHandler = new ElevatorCommandHandler(s_primaryElevatorSubsystem, s_innerElevatorSubsystem, s_armSubsystem);
    private final ClimbCommand ch_climbCommand = new ClimbCommand(s_climbSubsystem);
    private final IndexCommand ch_indexCommand = new IndexCommand(s_armSubsystem);
    
    private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem(driver0);
    //private teleopCommand teleCommand = new teleopCommand(controllerSubsystem, drivetrain, s_innerElevatorSubsystem);

    private alignToPose alignLeftReef = new alignToPose(drivetrain, s_blueVision1, true);

   // private alignToPose alignRightReef = new alignToPose(drivetrain, s_grayVision1, false);

    //** Commands **//
    // private PhotonCommand c_positionToRightPole = new PhotonCommand(s_grayPhotonVision, drivetrain, 0, 0.416, 0.17, () -> driver0.getRightY());
    // private PhotonCommand c_positionToLeftPole = new PhotonCommand(s_bluePhotonVision, drivetrain, 0, 0.416, -0.17, () -> driver0.getRightY());

    // private PhotonCommand1 c_positionToRightPole1 = new PhotonCommand1(s_grayPhotonVision, drivetrain, 0, 0.416, 0.17, () -> driver0.getRightY());
    // private PhotonCommand1 c_positionToLeftPole1 = new PhotonCommand1(s_bluePhotonVision, drivetrain, 0, 0.416, -0.17, () -> driver0.getRightY());
    // private final SwerveTeleop swerveTeleop = new SwerveTeleop(drivetrain, s_innerElevatorSubsystem, driver0);

    // private OdometryCommand c_positionToLeftPole = new OdometryCommand(drivetrain, s_grayVision, true);
    // private OdometryCommand c_positionToRightPole = new OdometryCommand(drivetrain, s_blueVision, false);

    // private PhotonCommand c_leftPoleAllignment = new PhotonCommand(s_grayPhotonVision, drivetrain, ReefPosition.LEFT);
    // private PhotonCommand c_rightPoleAllignment = new PhotonCommand(s_bluePhotonVision, drivetrain, ReefPosition.RIGHT);

    private final teleopCommand teleop = new teleopCommand(controllerSubsystem, drivetrain, s_innerElevatorSubsystem);
    //private final basicTeleop teleop2 = new basicTeleop(driver0, drivetrain, s_innerElevatorSubsystem);

//     private final OdoometryCommand odomCommand = new OdometryCommand(drivetrain, s_blueVision, false);
//     private final OdometryCommand1 odometryCommand1 = new OdometryCommand1(drivetrain);
//     private final AlignToReefTagRelative aRelative = new AlignToReefTagRelative(true, drivetrain);
//    //private final alignToPose sigma = new alignToPose(drivetrain, null);

    private final SequentialCommandGroup c_preIntakeToIntake = new SequentialCommandGroup(
        ch_elevatorCommandHandler.setArmState(ElevatorStates.INTAKE),
        ch_elevatorCommandHandler.setPrimaryElevatorState(ElevatorStates.INTAKE),
        //ch_elevatorCommandHandler.setElevators(ElevatorStates.INTAKE),
        ch_indexCommand.setIndexState(IndexStates.INTAKE),
        new WaitCommand(0.2),
        ch_elevatorCommandHandler.setInnerElevatorState(ElevatorStates.INTAKE),
        new WaitCommand(0.7),
        ch_indexCommand.setIndexState(IndexStates.STOP),
        ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
    
    private final SequentialCommandGroup c_score = new SequentialCommandGroup(
        new InstantCommand(() -> s_armSubsystem.setOverride(-0.3, true)),
        ch_indexCommand.setIndexState(IndexStates.OUTTAKE),
        new WaitCommand(0.7),
        new InstantCommand(() -> s_armSubsystem.setOverride(0, false)),
        ch_indexCommand.setIndexState(IndexStates.STOP),
        ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE)
    );//.addRequirements(s_primaryElevatorSubsystem, s_innerElevatorSubsystem, s_armSubsystem);


    private final basicTeleop basic = new basicTeleop(driver0, drivetrain, s_innerElevatorSubsystem);



    private final SequentialCommandGroup autoSigma = new SequentialCommandGroup( ch_elevatorCommandHandler.setElevators(ElevatorStates.L1), new driveComand(drivetrain), ch_indexCommand.setIndexState(IndexStates.OUTTAKE), new WaitCommand(2), ch_indexCommand.setIndexState(IndexStates.STOP), ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
   // private final SequentialCommandGroup autoSigma1 = new SequentialCommandGroup(ch_elevatorCommandHandler.setElevators(ElevatorStates.L4), new WaitCommand(1), sigma.until(() -> isdone(Timer.getFPGATimestamp())), c_score);
    //private final SequentialCommandGroup autoSigma1 = new SequentialCommandGroup(new driveComand(drivetrain), ch_elevatorCommandHandler.setElevators(ElevatorStates.L4), new WaitCommand(3), sigma.withDeadline(new WaitCommand(3)), c_score);


    public boolean isdone(double startTime) {
        if(Timer.getFPGATimestamp() - startTime > 3) {
            return true;
        }
        return false; 
    }

    public RobotContainer() {
        c_score.addRequirements(s_primaryElevatorSubsystem, s_innerElevatorSubsystem, s_armSubsystem);
        configureDriveBindings();
        configureDriver1Commands();
        configureAuto();
       // drivetrain.getPid
    }

    public boolean canRun() {
        if(!s_innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.STARTING_POSITION)) {
            return true;
        }
        return false;
    }


    // private boolean checkRun() {
    //     if(!s_beambreakSubsystem.getBeamBroken() && s_innerElevatorSubsystem.getInnerElevatorState().equals(ElevatorStates.PRE_INTAKE)) {
    //         return true;
    //     }
    //     return false;
    // }

    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            //     drive.withVelocityX(-Utilities.polynomialAccleration(controllerSubsystem.getPosY()) * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-Utilities.polynomialAccleration(controllerSubsystem.getPosX()) * MaxSpeed) // Drive left with negative X (left)
            //         .withRotationalRate(-Utilities.polynomialAccleration(driver0.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            // )
            //teleop
        basic
        );

        driver0.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver0.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver0.getLeftY(), -driver0.getLeftX()))));
        //driver0.x().whileTrue(spot1);
        
        // driver0.leftTrigger().onTrue(new InstantCommand(() -> s_innerElevatorSubsystem.setPrecision(true)));
        // driver0.leftTrigger().onFalse(new InstantCommand(() -> s_innerElevatorSubsystem.setPrecision(false)));

        // // driver0.rightTrigger().onTrue(new InstantCommand(() -> controllerSubsystem.setStrafeLeft(false)));
        // // driver0.rightTrigger().onFalse(new InstantCommand(() -> controllerSubsystem.setStrafe(false)));

        // driver0.rightBumper().onTrue(new InstantCommand(() -> controllerSubsystem.setPrecision(true)));
        // driver0.rightBumper().onFalse(new InstantCommand(() -> controllerSubsystem.setPrecision(false)));

         driver0.x().whileTrue(leftPhotonAlign);
     driver0.y().whileTrue(rightPhotonAlign);

        // reset the field-centric heading on left bumper press
        driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
     }

    private void configureDriver1Commands() {
        driver1.pov(90).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L4));
        driver1.pov(0).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L3));
        driver1.pov(270).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L2));
        driver1.pov(180).toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.L1));
       // driver1.pov(-1).toggleOnTrue(c_preIntakeToIntake.onlyIf(() -> checkRun()));
        driver1.b().toggleOnTrue(c_preIntakeToIntake);//.onlyIf(() -> checkRun()));
        driver1.x().onTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.STARTING_POSITION));
        driver1.a().toggleOnTrue(ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
        //driver1.b().toggleOnTrue(c_preIntakeToIntake.onlyIf(() -> checkRun()));
        driver1.y().onTrue(c_score.onlyIf(() -> canRun()));

        driver1.leftTrigger().onTrue(ch_indexCommand.setIndexState(IndexStates.OUTTAKE));
        driver1.leftTrigger().onFalse(ch_indexCommand.setIndexState(IndexStates.STOP));

        // driver1.leftTrigger().onTrue(new InstantCommand( () -> s_primaryElevatorSubsystem.setAddMotor(0.1)));
        // driver1.leftTrigger().onFalse(new InstantCommand( () -> s_primaryElevatorSubsystem.setAddMotor(0)));

        // driver1.rightTrigger().onTrue(new InstantCommand( () -> s_primaryElevatorSubsystem.setAddMotor(-0.1)));
        // driver1.rightTrigger().onFalse(new InstantCommand( () -> s_primaryElevatorSubsystem.setAddMotor(0)));

        // ONLY PRESS WHEN CLIMIBNG TO BALANCE WEIGHT
        // NEVER PRESS DURING MATCH 
        driver1.start().toggleOnTrue(ch_elevatorCommandHandler.setArmState(ElevatorStates.CLIMB));

        

        // driver1.rightTrigger().onTrue(new InstantCommand(() -> s_innerElevatorSubsystem.setAddSpeed(0.05)));
        // driver1.leftTrigger().onTrue(new InstantCommand(() -> s_innerElevatorSubsystem.setAddSpeed(-0.05)));

        // driver1.rightTrigger().onFalse(new InstantCommand(() -> s_innerElevatorSubsystem.setAddSpeed(0)));
        // driver1.leftTrigger().onFalse(new InstantCommand(() -> s_innerElevatorSubsystem.setAddSpeed(0)));

        driver1.leftBumper().onTrue(ch_climbCommand.setClimbState(ClimbStates.CLIMB));
        driver1.leftBumper().onFalse(ch_climbCommand.setClimbState(ClimbStates.STOP));

        driver1.rightBumper().onTrue(ch_climbCommand.setClimbState(ClimbStates.DROP));
        driver1.rightBumper().onFalse(ch_climbCommand.setClimbState(ClimbStates.STOP));

        // driver1.rightTrigger().onTrue(new InstantCommand(() -> s_climbSubsystem.setPidUseTrue()));
    }

    

    private void configureAuto() {
        NamedCommands.registerCommand("Score", c_score);
        NamedCommands.registerCommand("L4", ch_elevatorCommandHandler.setElevators(ElevatorStates.L4));
        NamedCommands.registerCommand("L1", ch_elevatorCommandHandler.setElevators(ElevatorStates.L1));
        NamedCommands.registerCommand("Pre intake", ch_elevatorCommandHandler.setElevators(ElevatorStates.PRE_INTAKE));
        NamedCommands.registerCommand("intake", c_preIntakeToIntake);
        NamedCommands.registerCommand("LineUp", new WaitCommand(0.1));    
        NamedCommands.registerCommand("Intake", ch_indexCommand.setIndexState(IndexStates.OUTTAKE));
        NamedCommands.registerCommand("Outake", ch_indexCommand.setIndexState(IndexStates.OUTTAKE));
        NamedCommands.registerCommand("Stop", ch_indexCommand.setIndexState(IndexStates.STOP));

         m_chooser = AutoBuilder.buildAutoChooser("1 Coral Straight");
         m_chooser.addOption("sigma boy ", autoSigma);
    //  m_chooser.addOption("sigma boy 1000", autoSigma1);
        // m_chooser.addOption("sigma boy1 ", autoSigma1);
         
        SmartDashboard.putData(m_chooser);
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public void resetDrive() {
         driver0.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
     }
    //     drivetrain.runOnce(() -> drivetrain.resetRotation(
    //         new Rotation2d(SwerveRequest.ForwardPerspectiveValue.valueOf(90).value)));
    // }

    public void resetPidgeon() {
        drivetrain.getPigeon2().setYaw(0);
       // drivetrain.setOperatorPerspectiveForward(new Rotation2d(180));
    }
}