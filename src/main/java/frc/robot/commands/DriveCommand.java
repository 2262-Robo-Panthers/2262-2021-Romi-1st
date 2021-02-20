// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveCommand extends CommandBase {

	private final RomiDrivetrain m_drivetrain;

	private final DoubleSupplier m_speedSupplier;
	private final DoubleSupplier m_rotateSupplier;

	/** Creates a new DriveCommand. */
	public DriveCommand(RomiDrivetrain drivetrain, DoubleSupplier speedSupplier, DoubleSupplier rotateSupplier) {
		m_drivetrain = drivetrain;
		m_speedSupplier = speedSupplier;
		m_rotateSupplier = rotateSupplier;
		addRequirements(m_drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_drivetrain.arcadeDrive(m_speedSupplier.getAsDouble(), m_rotateSupplier.getAsDouble());
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
