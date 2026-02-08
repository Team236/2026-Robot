package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.MainRoller;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;

public class AutoPivotShooter extends Command {
  private Swerve s_Swerve;
  private MainRoller mainRoller;
  private ShooterPivot shooterPivot;
  private double HUBX;
  private double HUBY;
  private double currentDist;
  private double targetRPM;
  private double targetRevs;

  public AutoPivotShooter(Swerve s_Swerve, MainRoller mainRoller, ShooterPivot shooterPivot) {
    // USING SWERVE FOR TAKING OVER SHOOTER
    this.s_Swerve = s_Swerve;
    this.mainRoller = mainRoller;
    this.shooterPivot = shooterPivot;

    addRequirements(mainRoller);
    addRequirements(shooterPivot);
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
    // FINDS DISTANCE THEN USES IT TO CALCULATE SPEEDS AND ANGLE
    currentDist = this.s_Swerve.getDistanceToHub(HUBX, HUBY);

    targetRPM = this.mainRoller.calculateRPM(currentDist);
    targetRevs = this.shooterPivot.calculateHoodAngle(currentDist);

    this.mainRoller.MainPID(targetRPM);
    this.shooterPivot.pidSetPosition(targetRevs);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}