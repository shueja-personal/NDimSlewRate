// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.GeneralSlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  GeneralSlewRateLimiter limiter = new GeneralSlewRateLimiter(1);
  XboxController joystick = new XboxController(0);
  Mechanism2d mechanism2d = new Mechanism2d(2.2, 2.2);
  MechanismRoot2d root;
  MechanismLigament2d actual;
  MechanismLigament2d limited;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SendableRegistry.add(mechanism2d, "stick");
    root = mechanism2d.getRoot("root", 1.1, 1.1);
    actual = root.append(new MechanismLigament2d("actual", 0.1, 0));
    limited = root.append(new MechanismLigament2d("limited", 0.1, 0));

  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putData(mechanism2d);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    Vector<N2> actualStickVector = VecBuilder.fill(joystick.getLeftX(), joystick.getLeftY());
    actual.setAngle(
      Units.radiansToDegrees(
        Math.atan2(
          actualStickVector.get(1, 0), 
          actualStickVector.get(0, 0)
        )
      )
    );
    actual.setLength(actualStickVector.normF());
    Matrix<N2, N1> limitedVector = limiter.calculate(actualStickVector);
    limited.setAngle(
      Units.radiansToDegrees(
        Math.atan2(
          limitedVector.get(1, 0), 
          limitedVector.get(0, 0)
        )
      )
    );
    limited.setLength(limitedVector.normF());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
