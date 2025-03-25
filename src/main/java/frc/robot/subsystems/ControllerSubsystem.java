package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConsts;
import frc.robot.States.ElevatorStates;

public class ControllerSubsystem extends SubsystemBase{
    CommandXboxController driver0;
 //InnerElevatorSubsystem elevatorSubsystem;

    double accX;
    double posX; // controller value

    double accY;
    double posY;

    double slow;

    public ControllerSubsystem(CommandXboxController driver0) {
        this.driver0 = driver0;

        accX = 0;
        posX = 0;
        accY = 0;
        posY = 0;

        slow = 1;
    }

    @Override
    public void periodic() {
        accX = getDriverLeftX();
        accY = getDriverLeftY();

        if(accX >= Math.abs(posX)) {
            posX = accX;
        }
        else {
            posX += (accX - posX) * ControllerConsts.FRIC;
        }

        if(accY >= Math.abs(posY)) {
            posY = accY;
        }
        else {
            posY += (accY - posY) * ControllerConsts.FRIC;
        }
        
        
        
            // posX = Math.max(-1 * slow, Math.min(1 * slow, posX * slow));
            // posY = Math.max(-1 * slow, Math.min(1 * slow, posY * slow));

            posX = Math.max(-1, Math.min(1, posX));
            posY = Math.max(-1, Math.min(1, posY));
        
        SmartDashboard.putNumber("posX", posX);
        SmartDashboard.putNumber("posY", posY);
        SmartDashboard.putNumber("slow", slow);
        // SmartDashboard.putNumber("addedFRIC", addedFRIC);
    }

    public void setSlow(double slow) {
        this.slow = slow;
    }

    public double getDriverLeftX() {
        return driver0.getLeftX();
    }

    public double getDriverLeftY() {
        return driver0.getLeftY();
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

}
