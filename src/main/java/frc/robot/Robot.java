/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import frc.config.Config;
import frc.controlloops.SwerveControl;
import frc.mechs.MechCollection;
import frc.swerve.FullSwerve;
import frc.swerve.NavXGyro;

public class Robot extends IterativeRobot {

	private SwerveControl swerve;
	private MechCollection mechs;
	private Autonomous auton;
	private Teleop teleop;

	@Override
	public void robotInit() {
		Config.start();
		swerve = new SwerveControl(new FullSwerve(new NavXGyro()));
		swerve.start();
		mechs = new MechCollection();
		auton = new Autonomous(swerve, mechs);
		teleop = new Teleop(swerve, mechs);
	}

	@Override
	public void autonomousInit() {
		swerve.enable();
		auton.init();
	}

	@Override
	public void autonomousPeriodic() {
		auton.periodic();
	}

	@Override
	public void teleopInit() {
		swerve.enable();
		teleop.init();
	}

	@Override
	public void teleopPeriodic() {
		teleop.periodic();
	}

	@Override
	public void disabledInit() {
		swerve.disable();
	}

	@Override
	public void testInit() {
		swerve.enable();
	}

	@Override
	public void testPeriodic() {
	}
}
