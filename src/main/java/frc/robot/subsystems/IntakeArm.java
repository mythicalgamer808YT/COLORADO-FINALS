package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkFlexUtil;
import frc.robot.Constants;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IndexStates;

public class IntakeArm extends SubsystemBase {
    private final SparkFlex armMotor;
    private final SparkMax indexingMotor;
    private final DutyCycleEncoder armEncoder;
    private final PIDController armPID; 
    private final ArmFeedforward armFeedforward;

    private IndexStates indexState;
    private ElevatorStates elevatorState;

    private double armPosition;
    private double armSpeed;
    
    private boolean override;
    private boolean inBounds; 

    private double addSpeed;

    public IntakeArm() {
        armMotor = new SparkFlex(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(3);
        indexingMotor = new SparkMax(Constants.ArmConstants.INDEX_MOTOR_ID, MotorType.kBrushless);

        armPID = new PIDController(Constants.ArmConstants.KP, Constants.ArmConstants.KI, Constants.ArmConstants.KD);
        armFeedforward = new ArmFeedforward(Constants.ArmConstants.KS, Constants.ArmConstants.KG, Constants.ArmConstants.KV);

        indexState = IndexStates.STOP;;
        elevatorState = ElevatorStates.STARTING_POSITION;

        armPosition = getArmAngle();
        armSpeed = 0;
        override = false;
        inBounds = false;

        addSpeed = 0;

        SparkFlexUtil.setSparkFlexBusUsage(armMotor, SparkFlexUtil.Usage.kAll, IdleMode.kBrake, false, false);
    }

    @Override
    public void periodic() {
        armPosition = getArmAngle();
        
        if(armPosition >= ElevatorStates.MAX.armAngle) {
            // posotive is up
            armSpeed = -0.1;
            inBounds = false;
        } else if(armPosition <= ElevatorStates.MIN.armAngle) {
            armSpeed = 0.1;
            inBounds = false;
        } else {
            if(!override) {
                armSpeed = armPID.calculate(armPosition) + 
                    armFeedforward.calculate(Units.degreesToRadians(armPosition), armMotor.getAbsoluteEncoder().getVelocity());
                inBounds = true;
            }
        }

         

        armMotor.set(armSpeed);
        setSmartDashboardValues();
    }

    public void setArmState(ElevatorStates state) {
        elevatorState = state;
        armPID.setSetpoint(state.armAngle);
    }

    // public void setAddSpeed(double addSpeed) {
    //     this.addSpeed = addSpeed;
    // }

    public void setOverride(double armSpeed, boolean override) {
        this.override = override;
        this.armSpeed = armSpeed;
    }

    public void setIndexState(IndexStates state) {
        indexState = state;
        indexingMotor.set(state.speed);
    }
    
    public double getArmAngle() {
        return Units.rotationsToDegrees(armEncoder.get()) + 33;
    }

    private void setSmartDashboardValues() {
        SmartDashboard.putNumber("Arm Subsystem goal angle", elevatorState.armAngle);
        SmartDashboard.putNumber("Arm Subsystem position", armPosition);
        SmartDashboard.putNumber("Arm Subsystem motor speed", armSpeed);     
        
        SmartDashboard.putString("Arm Subsytem index state", indexState.toString());    
        SmartDashboard.putString("Arm Subsystem elevator state", elevatorState.toString());

        SmartDashboard.putBoolean("Arm Subsystem in bounds", inBounds);
        SmartDashboard.putBoolean("Arm Subsystem override", override);
    }
}
