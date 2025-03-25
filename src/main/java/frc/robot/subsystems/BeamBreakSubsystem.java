package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
    private final DigitalInput beamBreak;

    private boolean beamBroken;

    public BeamBreakSubsystem() {
        beamBreak = new DigitalInput(2);
        beamBroken = false;
    }

    @Override
    public void periodic() {
        if (beamBreak.get()) {
            beamBroken = true;
        } else {
            beamBroken = false;
        }

        SmartDashboard.putBoolean("Beam Broken", beamBroken);
    }

    public boolean getBeamBroken() {
        return beamBroken;
    }
}
