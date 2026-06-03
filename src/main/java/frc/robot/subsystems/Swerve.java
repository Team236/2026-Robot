package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator m_poseEstimator;
    public double poseAngle;
    public double poseForwardDistance;
    public double poseSideDistance;
    public PIDController pidControllerForTrackingOutput;
    public PIDController pidForShaking;

    private double cachedHubX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
    private double cachedHubY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;

    private double pipeline = 0; 
    private double tv;
    public Pose2d poseLL;
    public Pose2d targetPose;

    private Field2d field = new Field2d();

    private PIDController PPHeadingPIDController;

    public SwerveControllerCommand currentSwerveControllerCommand;
    public Trajectory currentTrajectory;

    public double[] driveTargetingValues;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants), 
            new SwerveModule(2, Constants.Swerve.Mod2.constants), 
            new SwerveModule(3, Constants.Swerve.Mod3.constants) 
        };
        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(), 
                mSwerveMods[1].getPosition(),
                mSwerveMods[2].getPosition(),
                mSwerveMods[3].getPosition()  
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), 
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30)) 
        );

        SmartDashboard.putData("Field", field);


        SmartDashboard.putString("Starting swerve and pathplanner", "yes");
        SmartDashboard.putNumber("PPOverrideHeading target angle", 0);
        SmartDashboard.putNumber("PPOverrideHeading PID output", 0);

        RobotConfig config = null; 
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        
        if (config == null) {
            System.out.println("PathPlanner RobotConfig settings null, may have errored");
        } else {
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> driveWithChassisSpeeds(speeds),
                new PPHolonomicDriveController(
                    Constants.PathPlanner.TRANSLATION_PID_CONSTANTS,
                    Constants.PathPlanner.ROTATION_PID_CONSTANTS 
                ),
                config,
                () -> { 
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red; 
                    }
                    return false;
                },
                this
            );
        }

        pidControllerForTrackingOutput = new PIDController(
        Constants.Targeting.AUTO_ROTATE_KP,
        0.0,
        Constants.Targeting.AUTO_ROTATE_KD);

        pidForShaking = new PIDController(
        Constants.Targeting.AUTO_ROTATE_KP + 1.0,
        0.0,
        Constants.Targeting.AUTO_ROTATE_KD);

        pidControllerForTrackingOutput.enableContinuousInput(-Math.PI, Math.PI);
        pidForShaking.enableContinuousInput(-Math.PI, Math.PI);
        pidControllerForTrackingOutput.setTolerance(Math.toRadians(0.1));
        pidControllerForTrackingOutput.setSetpoint(0.0);
        pidForShaking.setTolerance(Math.toRadians(0.1));
        pidForShaking.setSetpoint(0.0);

        PPHeadingPIDController = new PIDController(
            Constants.PathPlanner.ROTATION_PID_CONSTANTS.kP,
            Constants.PathPlanner.ROTATION_PID_CONSTANTS.kI,
            Constants.PathPlanner.ROTATION_PID_CONSTANTS.kD
        );
        PPHeadingPIDController.enableContinuousInput(-Math.PI, Math.PI);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            cachedHubX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
            cachedHubY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
        } else {
            cachedHubX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
            cachedHubY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
        }
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        var alliance = DriverStation.getAlliance();

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    alliance.isPresent() && alliance.get() == Alliance.Red ? -translation.getX() : translation.getX(), 
                                    alliance.isPresent() && alliance.get() == Alliance.Red ? -translation.getY() : translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

   
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public Command followPathCommand(String pathName) {
        try {
            SmartDashboard.putNumber("RobotPoseX before path start", this.getPose().getX());
            SmartDashboard.putNumber("RobotPoseY before path start", this.getPose().getY());
            SmartDashboard.putNumber("RobotPoseRotation before path start", this.getPose().getRotation().getDegrees());

            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            
            Pose2d bluePathStartingPose = path.getStartingHolonomicPose().get();
            
            if (DriverStation.getAlliance().get() == Alliance.Red) { 
                this.resetPose(FlippingUtil.flipFieldPose(bluePathStartingPose));
            } else  {
                this.resetPose(path.getStartingHolonomicPose().get());
            }

            SmartDashboard.putNumber("RobotPoseX AFTER", this.getPose().getX());
            SmartDashboard.putNumber("RobotPoseY AFTER", this.getPose().getY());
            SmartDashboard.putNumber("RobotPoseRotation AFTER", this.getPose().getRotation().getDegrees());

            Waypoint pathStartWaypoint = path.getWaypoints().get(0);
            SmartDashboard.putNumber("Path start  X", pathStartWaypoint.anchor().getX());
            SmartDashboard.putNumber("Path start  Y", pathStartWaypoint.anchor().getY());
            SmartDashboard.putNumber("Path start  Rotation", path.getStartingHolonomicPose().get().getRotation().getDegrees());

            Waypoint pathEndWaypoint = path.getWaypoints().get(path.getWaypoints().size() - 1);
            SmartDashboard.putNumber("Path end  X", pathEndWaypoint.anchor().getX());
            SmartDashboard.putNumber("Path end  Y", pathEndWaypoint.anchor().getY());
            SmartDashboard.putNumber("Path end  Rotation", path.getGoalEndState().rotation().getDegrees());

            FieldObject2d start = field.getObject("PathStart");
            start.setPose(pathStartWaypoint.anchor().getX(), pathStartWaypoint.anchor().getY(), path.getIdealStartingState().rotation());
            
            FieldObject2d end = field.getObject("PathEnd");
            end.setPose(pathEndWaypoint.anchor().getX(), pathEndWaypoint.anchor().getY(), path.getGoalEndState().rotation());

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command followPathCommandNoReset(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command followPathCommandRobotStartingPose(String pathName) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromPathFile(pathName);
            PathConstraints constraints = originalPath.getGlobalConstraints();

            List<Waypoint> newWaypoints = originalPath.getWaypoints();
            newWaypoints.set(0, PathPlannerPath.waypointsFromPoses(this.getPose()).get(0)); 

            PathPlannerPath path = new PathPlannerPath(
                newWaypoints, 
                constraints, 
                null, 
                originalPath.getGoalEndState()
            );
            
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command getClimbTargetingPath() {
        try {
            Pose2d robotPoseBlue;
            
        
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) { 
                robotPoseBlue = FlippingUtil.flipFieldPose(this.getPose());
            } else  {
                robotPoseBlue = this.getPose();    
            }
            
            PathConstraints constraints = PathPlannerPath.fromPathFile("climb-targeting").getGlobalConstraints();
            Pose2d pathEndPose;

            if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
                pathEndPose = new Pose2d(1.146 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 3.302, Rotation2d.fromDegrees(180)); 
            } else {
                pathEndPose = new Pose2d(1.146 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 4.162, Rotation2d.fromDegrees(180));
            }

            Rotation2d splineHeading = pathEndPose.getTranslation().minus(robotPoseBlue.getTranslation()).getAngle();
            Pose2d startPoseForSpline = new Pose2d(robotPoseBlue.getTranslation(), splineHeading);

        
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPoseForSpline, 
                pathEndPose 
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            );
            
            return AutoBuilder.followPath(path);
            
        } catch (Exception e) {
            DriverStation.reportError("Error in getClimbTargetingPath: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    public Command getClimbTargetingPathNew() {
        try {
            Pose2d robotPoseBlue;
            
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) { 
                robotPoseBlue = FlippingUtil.flipFieldPose(this.getPose());
            } else  {
                robotPoseBlue = this.getPose();    
            }
            
            Pose2d pathEndPoseBlue;
            if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
                pathEndPoseBlue = new Pose2d(1.640, 3.304, Rotation2d.fromDegrees(0));
            } else {
                pathEndPoseBlue = new Pose2d(1.640, 4.162, Rotation2d.fromDegrees(0));
            }

            Pose2d pathEndPoseAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? FlippingUtil.flipFieldPose(pathEndPoseBlue) : pathEndPoseBlue;
            PathConstraints constraints = new PathConstraints(
                2.25,
                3.0,
                540.0,
                720.0,
                12.0
            );

            if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
                return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("tr0_to_cr0"), constraints);
            } else { 
                return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("tl0_to_cl0"), constraints);
            }

        } catch (Exception e) {
            DriverStation.reportError("Error in getClimbTargetingPathNew: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    
    
    public Command followPathCommandRobotRelative(String pathName) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromPathFile(pathName);

            PathConstraints constraints = originalPath.getGlobalConstraints(); 
            IdealStartingState originalStartingState = originalPath.getIdealStartingState();
            GoalEndState originalGoalEndState = originalPath.getGoalEndState();
            
            Pose2d pathInitialPose = originalPath.getStartingHolonomicPose().get(); 
            List<Pose2d> newPoses = new ArrayList<>();
            newPoses.add(this.getPose()); 

            int i = 0;
            for (Pose2d pose : originalPath.getPathPoses()) {
                if (i > 0) { 
                    newPoses.add(this.getPose().transformBy(pose.minus(pathInitialPose))); 
                }

                i++;
            }

            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(newPoses),
                constraints, 
                new IdealStartingState(originalStartingState.velocity(), originalStartingState.rotation().plus(getHeading().minus(pathInitialPose.getRotation()))), 
                new GoalEndState(originalGoalEndState.velocity(), originalGoalEndState.rotation().plus(getHeading().minus(pathInitialPose.getRotation())))
            );

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

        public Command getFinishClimbCommand() {
        try {
            Pose2d robotPoseBlue;
            
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) { 
                robotPoseBlue = FlippingUtil.flipFieldPose(this.getPose());
            } else  {
                robotPoseBlue = this.getPose();    
            }
            
            PathConstraints constraints = PathPlannerPath.fromPathFile("finish-climb").getGlobalConstraints();
            Pose2d pathEndPose;

            if (robotPoseBlue.getY() < Constants.Targeting.BLUE_TOWER_CENTER_Y_METERS) {
                pathEndPose = new Pose2d(.955 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 3.302, Rotation2d.fromDegrees(180)); 
            } else {
                pathEndPose = new Pose2d(.955 + Constants.Targeting.ROBOT_WIDTH_METERS / 2.0, 4.162, Rotation2d.fromDegrees(180));
            }

            Rotation2d splineHeading = pathEndPose.getTranslation().minus(robotPoseBlue.getTranslation()).getAngle();
            Pose2d startPoseForSpline = new Pose2d(robotPoseBlue.getTranslation(), splineHeading);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startPoseForSpline, 
                pathEndPose 
            );

            PathPlannerPath path = new PathPlannerPath(
                waypoints, 
                constraints, 
                null, 
                new GoalEndState(0.0, Rotation2d.fromDegrees(0))
            );
            
            return AutoBuilder.followPath(path);
            
        } catch (Exception e) {
            DriverStation.reportError("Error in getClimbTargetingPath: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
    

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = getModuleStates();
        ChassisSpeeds fieldRelFromStates = Constants.Swerve.swerveKinematics.toChassisSpeeds(states);
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelFromStates, getHeading());
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
       return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void getLLPose() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()  && (tv == 1)) {
            if (ally.get() == Alliance.Red){
            poseLL = LimelightHelpers.getBotPose2d_wpiRed("limelight");

            }
            if (ally.get() == Alliance.Blue){
            poseLL = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            
            }   
        }

    }

    public void getTargetPose(Pose2d targetPose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Blue){
            this.targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().plus(new Rotation2d(Math.PI)));
        }
        if (ally.get() == Alliance.Red){ 
            this.targetPose = targetPose; 
        }   

    }

    public void resetFieldPoseWithTarget() {
        if (targetPose != null) {
            resetPose(targetPose);
        }
    }

    public void resetLLPose() {
        if (poseLL != null) {
            resetPose(poseLL);
        }
    }

    public void MegaTag2UpdateOdometry() {

        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        boolean useMegaTag2 = true;

        if (useMegaTag2 == false)
        {
            for (String limelightName : Constants.Targeting.CAMERA_NAMES) {
                boolean useThisEstimate = true;
                LimelightHelpers.PoseEstimate estimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

                if (estimateMT1 == null) {
                    break; 
                }

                if((estimateMT1.tagCount == 1 && estimateMT1.rawFiducials.length == 1))
                {   
                    if(estimateMT1.rawFiducials[0].ambiguity > .7) { useThisEstimate = false; }
                    if(estimateMT1.rawFiducials[0].distToCamera > 3) { useThisEstimate = false; }
                }

                if(estimateMT1.tagCount == 0) { 
                    useThisEstimate = false; 
                }

                if(useThisEstimate) {
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,0.01));
                    m_poseEstimator.addVisionMeasurement(
                        estimateMT1.pose,
                        estimateMT1.timestampSeconds);
                }
            }       
        } else if (useMegaTag2) {
            for (String limelightName : Constants.Targeting.CAMERA_NAMES) {   
                boolean useThisEstimate = true;
                LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                
                if (estimateMT2 == null) {
                    break;
                }
                
                if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720 || estimateMT2.tagCount == 0)
                {
                    useThisEstimate = false;
                }

                if (useThisEstimate)
                {
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 999999));
                    m_poseEstimator.addVisionMeasurement(
                        estimateMT2.pose,
                        estimateMT2.timestampSeconds);
                }
            }
        }
    }

    public double calculateTargetingPID (double HUBX, double HUBY) {
      Pose2d currentPose = getPose();

      double dx = HUBX - Units.metersToInches(currentPose.getX());
      double dy = HUBY - Units.metersToInches(currentPose.getY());
      double targetAngle = Math.atan2(dy, dx);

      double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
      return pidOutput;
    }

    public double calculateFaceAlliancePID() {
        Pose2d currentPose = getPose();
        double targetAngle;
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            targetAngle = 0; 
        } else {
            targetAngle = Math.toRadians(180); 
        }

        double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
        return pidOutput;
    }

    public double calculateTargetingAutoPID(double targetAngle) {
        Pose2d currentPose = getPose();
        double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
    
        return pidOutput;
    }

    public double getPPOverrideHeadingFeedback() {
        Pose2d currentPose = getPose();

        ChassisSpeeds robotChassisSpeeds = getChassisSpeeds();
        ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, getHeading());

        double targetAngle = new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond).getAngle().getRadians();
        SmartDashboard.putNumber("PPOverrideHeading target angle", targetAngle);

        SmartDashboard.putNumber("PPOverrideHeading PID output", PPHeadingPIDController.calculate(currentPose.getRotation().getRadians(), targetAngle));
        return PPHeadingPIDController.calculate(currentPose.getRotation().getRadians(), targetAngle);
    }

    public Command getPPOverrideHeadingCommand() {
        return 
            new Command() {
                @Override
                public void initialize() {
                    PPHolonomicDriveController.overrideRotationFeedback(Swerve.this::getPPOverrideHeadingFeedback);
                }

                @Override
                public void end(boolean interrupted) {
                    PPHolonomicDriveController.clearRotationFeedbackOverride();
                }
            };
    }

    public Command getPPTargetingCommand() {
        return
            new Command() {
                double hubX;
                double hubY;

                @Override
                public void initialize() {
                    var alliance = DriverStation.getAlliance();

                    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        hubX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
                        hubY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
                    } else {
                        hubX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
                        hubY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
                    }

                    PPHolonomicDriveController.overrideRotationFeedback(() -> calculateTargetingPID(hubX, hubY));
                }   

                @Override
                public void end(boolean interrupted) {
                    PPHolonomicDriveController.clearRotationFeedbackOverride();
                }
            };
    }

    public double getAngleOfHub (double HUBX, double HUBY) {
        Pose2d currentPose = getPose();
        double dx = HUBX - Units.metersToInches(currentPose.getX());
        double dy = HUBY - Units.metersToInches(currentPose.getY());
        double angle = Math.atan2(dy, dx);

        return angle;
    }

    public double getHubX() {
        return cachedHubX;
    }

    public double getHubY() {
        return cachedHubY;
    }

   public double getDistanceToHub() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    double HUBX;
    double HUBY;

    if (alliance.isPresent()) {
           if (alliance.get() == Alliance.Red) {
                HUBX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
                HUBY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
           } else {
                HUBX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
                HUBY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
           }
           
            Pose2d currentPose = getPose();
            double dx = HUBX - Units.metersToInches(currentPose.getX());
            double dy = HUBY - Units.metersToInches(currentPose.getY());
            double distance = Math.hypot(dy, dx);
        
            return distance;
       }
      return 0;
   }

   public double pidCalculateAngle (double targetAngle) {
   Pose2d currentPose = getPose();
   double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
  
   return pidOutput;
  }

    public double[] getRotationMoving () {
        final double TIME_OF_FLIGHT = 1.2;

        ChassisSpeeds robotChassisSpeeds = getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, getHeading());

        Pose2d currentPose2d = getPose();

        double hubXMeters = Units.inchesToMeters(getHubX());
        double hubYMeters = Units.inchesToMeters(getHubY());

        double virtualX = hubXMeters - (fieldSpeeds.vxMetersPerSecond * TIME_OF_FLIGHT);
        double virtualY = hubYMeters - (fieldSpeeds.vyMetersPerSecond * TIME_OF_FLIGHT);
        
        Translation2d virtualGoal = new Translation2d(virtualX, virtualY);

        double finalDistanceMeters = currentPose2d.getTranslation().getDistance(virtualGoal);
        double finalDistanceInches = Units.metersToInches(finalDistanceMeters);

        Rotation2d finalRot = new Rotation2d(
            virtualGoal.getX() - currentPose2d.getX(),
            virtualGoal.getY() - currentPose2d.getY()
        );

        SmartDashboard.putNumber("AimOnMove/VirtualGoalX (m)", virtualGoal.getX());
        SmartDashboard.putNumber("AimOnMove/VirtualGoalY (m)", virtualGoal.getY());
        SmartDashboard.putNumber("AimOnMove/FinalAngle (deg)", finalRot.getDegrees());
        SmartDashboard.putNumber("AimOnMove/DistanceToHub (in)", finalDistanceInches);
        SmartDashboard.putNumber("AimOnMove/FieldVx (m/s)", fieldSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("AimOnMove/FieldVy (m/s)", fieldSpeeds.vyMetersPerSecond);

        return new double[] {finalDistanceInches, finalRot.getRadians()};
    }

       public boolean inNeutralZone() {
       Pose2d currentPose = getPose();

       Optional<Alliance> alliance = DriverStation.getAlliance();

       if (alliance.isPresent()) {
           if (alliance.get() == Alliance.Red) {
               return currentPose.getX() < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_TOLERANCE_X);
           } else {
               return currentPose.getX() > Units.inchesToMeters(Constants.Targeting.BLUE_NEUTRAL_TOLERANCE_X);
           }
       }
       return false;
   }

    public double getAllianceWallHeading(double HUBX, double HUBY, double offsetRadians) {
        Pose2d currentPose = getPose();
        boolean shouldPass = false;
        var alliance = DriverStation.getAlliance();
        double finalTargetAngle;

        if (alliance.isPresent()){
            if (alliance.get() == Alliance.Red) {
                if (currentPose.getX() < Units.inchesToMeters(Constants.Targeting.RED_NEUTRAL_TOLERANCE_X)) {
                    shouldPass = true;
                }
            } else {
                if (currentPose.getX() > Units.inchesToMeters(Constants.Targeting.BLUE_NEUTRAL_TOLERANCE_X)) {
                    shouldPass = true;
                }
            }
        }

        if (shouldPass) {
            
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), 0);
            } else {
                return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), Math.toRadians(180));
            }
        }

        double dx = HUBX - Units.metersToInches(currentPose.getX());
        double dy = HUBY - Units.metersToInches(currentPose.getY());
        double targetAngle = Math.atan2(dy, dx);

        double angleDifference = MathUtil.angleModulus(targetAngle - currentPose.getRotation().getRadians());
        double amplitudeRadians = Constants.Targeting.SHAKE_SPREAD / Math.max(getDistanceToHub(), 1.0);

        double dynamicTolerance = Math.toRadians(2.0) + Math.abs(amplitudeRadians);

        if (Math.abs(angleDifference) < dynamicTolerance) {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.33);
            RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.33);

            finalTargetAngle = targetAngle + offsetRadians;

            return pidForShaking.calculate(currentPose.getRotation().getRadians(), finalTargetAngle);
        } else {
            RobotContainer.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
            RobotContainer.auxController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);

            finalTargetAngle = targetAngle;

            return pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), finalTargetAngle);
        }
    }

    public double getShakingOffset() {
        double shakeFrequency = Constants.Targeting.SHAKE_FREQUENCY;
        double shakeSpread = Constants.Targeting.SHAKE_SPREAD;

        double distanceInches = Math.max(getDistanceToHub(), 1.0);

        double amplitudeRadians = shakeSpread / distanceInches;

        double time = Timer.getFPGATimestamp();
        return amplitudeRadians * Math.sin(2 * Math.PI * shakeFrequency * time);
    }

   public boolean inNeutralMid(double yPose) {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return yPose > Constants.Targeting.RED_NEUTRAL_MID_RIGHT && 
                       yPose < Constants.Targeting.RED_NEUTRAL_MID_LEFT;
            } else if (alliance.get() == Alliance.Blue) {
                return yPose > Constants.Targeting.BLUE_NEUTRAL_MID_RIGHT && 
                       yPose < Constants.Targeting.BLUE_NEUTRAL_MID_LEFT;
            }
        }
        return false;
    }


    public double getXtoHub(){
    Pose2d currentPose = getPose();

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
        if (alliance.get() == Alliance.Red) {
            return Math.abs(Units.metersToInches(currentPose.getX()) - Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X);
        } else {
            return Math.abs(Units.metersToInches(currentPose.getX()) - Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X);
        }
    }
        return 0.0;
    }

    @Override
    public void periodic(){
        MegaTag2UpdateOdometry();
        SmartDashboard.putNumber("auto pivot desired rotation (red)", Units.radiansToDegrees(getAngleOfHub(Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X, Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y)));
        SmartDashboard.putNumber("auto pivot current rotation (red)", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Distance to hub", getDistanceToHub());
        SmartDashboard.putNumber("** RobotPoseX (Estimator)", Units.metersToInches( m_poseEstimator.getEstimatedPosition().getX()));
        SmartDashboard.putNumber("** RobotPoseY (Estimator)", Units.metersToInches( m_poseEstimator.getEstimatedPosition().getY()));
        SmartDashboard.putNumber("MegaTag2Rotation (Estimator)", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
       SmartDashboard.putBoolean("Is in nuetral", inNeutralZone());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder degrees", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle degrees", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        field.setRobotPose(getPose());
    }
}