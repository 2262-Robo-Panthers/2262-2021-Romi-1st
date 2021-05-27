/*
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoorCommand extends CommandBase{
    private final PWM m_doorPWM;

    private final BooleanSupplier m_runForward;
    private final BooleanSupplier m_runBackward;

    public DoorCommand(PWM doorPWM, BooleanSupplier runForward, BooleanSupplier runBackward) {
        m_doorPWM = doorPWM;
        m_runForward = runForward;
        m_runBackward = runBackward;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
        if(m_runForward.getAsBoolean()) m_doorPWM.setSpeed(1);
        else if(m_runBackward.getAsBoolean()) m_doorPWM.setSpeed(-1);
        else m_doorPWM.setSpeed(0);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
*/
