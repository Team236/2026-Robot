package frc.robot.commands.Targeting;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AimWhileMoving extends Command {
    Swerve s_Swerve;
    Pose2d currentPose2d;
    double angleToHub;
    double distanceToHub;
    ChassisSpeeds robotChassisSpeeds;
    double timeOfFlight;
    double HUBX;
    double HUBY;

  public AimWhileMoving(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();

    // ALIGENCE SELECTION FROM DRIVER STATION
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      HUBX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
      HUBY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
    } else {
      HUBX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
      HUBY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
    }
  }

  @Override
  public void execute() {
    currentPose2d = s_Swerve.getPose();
    distanceToHub = s_Swerve.getDistanceToHub(HUBX, HUBY);
    angleToHub = s_Swerve.getAngleToHub(HUBX, HUBY);

    timeOfFlight = Constants.Targeting.timeMap.get(distanceToHub);

    Translation2d virtualGoal = new Translation2d();

    for (int i = 0; i < 2; i++) {
    // Calculate the Virtual Goal
    // Target position - (Robot Velocity * Time)
    double virtualX = HUBX - (robotChassisSpeeds.vxMetersPerSecond * timeOfFlight);
    double virtualY = HUBY - (robotChassisSpeeds.vyMetersPerSecond * timeOfFlight);
    
    virtualGoal = new Translation2d(virtualX, virtualY);

    // Recalculate distance to this new virtual point
    double newDistance = currentPose2d.getTranslation().getDistance(virtualGoal);
    
    // Update time of flight for next loop
    timeOfFlight = Constants.Targeting.timeMap.get(newDistance);
    }

    double finalRotation = Math.atan2(
    virtualGoal.getY() - currentPose2d.getY(),
    virtualGoal.getX() - currentPose2d.getX()
    );
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}