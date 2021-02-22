// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.RomiDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();

	private final Joystick m_joystick = new Joystick(0);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	private Command generateRamseteCommand() {

		var autoVoltageConstraint =
			new DifferentialDriveVoltageConstraint(Constants.kDriveFeedforward, Constants.kDriveKinematics, 10); // Max voltage is <12 to account for current draw

		TrajectoryConfig config =
			new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
								 Constants.kMaxAccelerationMetersPerSecondSquared)
				.setKinematics(Constants.kDriveKinematics)
				.addConstraint(autoVoltageConstraint);

		// This trajectory can be modified to suit your purposes
		// Note that all coordinates are in meters, and follow NWU conventions.
		// If you would like to specify coordinates in inches (which might be easier
		// to deal with for the Romi), you can use the Units.inchesToMeters() method
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			new Pose2d(), Constants.kInteriorWaypoints, Constants.kEndPose, config);

		RamseteCommand ramseteCommand = new RamseteCommand(
			exampleTrajectory,
			m_romiDrivetrain::getPose,
			new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
			Constants.kDriveFeedforward,
			Constants.kDriveKinematics,
			m_romiDrivetrain::getWheelSpeeds,
			new PIDController(Constants.kPDriveVel, Constants.kIDriveVel, Constants.kDDriveVel),
			new PIDController(Constants.kPDriveVel, Constants.kIDriveVel, Constants.kDDriveVel),
			m_romiDrivetrain::tankDriveVolts,
			m_romiDrivetrain);

		m_romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());

		// Set up a sequence of commands
		// First, we want to reset the drivetrain odometry
		return new InstantCommand(() -> m_romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_romiDrivetrain)
			// next, we run the actual ramsete command
			.andThen(ramseteCommand)

			// Finally, we make sure that the robot stops
			.andThen(new InstantCommand(m_romiDrivetrain::stopDrivetrain, m_romiDrivetrain));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		m_romiDrivetrain.setDefaultCommand(getTeleopCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return generateRamseteCommand();
	}

	public Command getTeleopCommand() {
		return new DriveCommand(m_romiDrivetrain, () -> -m_joystick.getY(), m_joystick::getX, m_joystick::getTrigger);
	}
}
