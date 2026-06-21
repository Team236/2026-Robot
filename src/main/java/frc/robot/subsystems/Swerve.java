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

/**
 * The Swerve subsystem controls the drivetrain and manages the robot's odometry (its understanding 
 * of its location on the field). It interfaces with the gyro, the swerve modules, PathPlanner for 
 * autonomous driving, and vision systems for targeting.
 */

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    // The PoseEstimator merges wheel data, gyro data, and camera data to guess the robot's X, Y, and Rotation on the field.
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

    // A virtual representation of the field sent to the driver station dashboard.
    private Field2d field = new Field2d();

    private PIDController PPHeadingPIDController;

    public SwerveControllerCommand currentSwerveControllerCommand;
    public Trajectory currentTrajectory;

    public double[] driveTargetingValues;

    /**
     * Initializes the Swerve subsystem, configures the gyro, sets up the pose estimator, 
     * and links the drivetrain to PathPlanner for autonomous control.
     */

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
    
        // A swerve drive has 4 independent modules. Each has a drive motor to go 
        // forward/back, and a steering motor to pivot the wheel. We initialize them here using
        // the hardware IDs defined in Constants.
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants), 
            new SwerveModule(2, Constants.Swerve.Mod2.constants), 
            new SwerveModule(3, Constants.Swerve.Mod3.constants) 
        };

        // The Pose Estimator is initialized with "Standard Deviations" (the VecBuilder arrays).
        // This tells the math model how much to "trust" the wheels vs. how much to trust the vision cameras.
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
            // This block hands control over to PathPlanner. We give PathPlanner 
            // "get" methods so it knows where the robot is, and "drive" methods so it can 
            // command the motors during autonomous routines.
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

        // PID Controllers are set up here to handle auto-aiming. 
        // continuousInput(-Pi, Pi) ensures the robot takes the shortest path when spinning 
        // (e.g., spinning 10 degrees left instead of 350 degrees right).
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

        // Sets up the target location depending on what side of the field we are playing on.
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            cachedHubX = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_X;
            cachedHubY = Constants.Targeting.RED_ALLIANCE_HUB_CENTER_Y;
        } else {
            cachedHubX = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_X;
            cachedHubY = Constants.Targeting.BLUE_ALLIANCE_HUB_CENTER_Y;
        }
    }

    /**
     * The primary method used to drive the robot.
     * @param translation The desired X and Y speed.
     * @param rotation The desired rotational speed.
     * @param fieldRelative If true, pushing "up" on the joystick moves the robot away from the driver regardless of where the front of the robot is pointing.
     * @param isOpenLoop If true, runs motors based on raw percentage instead of using internal PID velocity control.
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        var alliance = DriverStation.getAlliance();

        // SwerveDriveKinematics contains the complex trigonometry required to convert 
        // a simple "move forward and twist" joystick command into 4 specific speeds and 4 specific 
        // angles for our independent wheel modules.
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

        // Prevents the math from requesting a wheel speed higher than the physical motors can achieve.
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

    // AUTONOMOUS PATHING METHODS
    // The following followPath... methods load pre-drawn paths from the PathPlanner GUI,
    // apply rules for whether the robot is on the Red or Blue alliance, and pass them to the AutoBuilder.

    /**
     * Follows a path from the PathPlanner GUI, resetting the robot's odometry to the starting point of the path. This means
     * that if the robot is not physically at the correct starting position, its Pose will still be reset to the start point.
     * @param pathName
     * 
     */
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

    /**
     * This method returns a command which will command the robot to first approach the starting position of the path (if it isn't already there)
     * and then follow and run the path as normal. This is the most basic PathPlanner implementation.
     * @param pathName
     * 
     */
    public Command followPathCommandNoReset(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * This method is similar to followPathCommandNoReset, but it also MODIFIES the path's starting waypoint to be the robot's current position.
     * This allows the robot to "grab" the path from its current location and follow it, even if it didn't start at the path's original starting point. 
     * This can be useful in situations where the robot might be slightly off from the ideal starting pose of the path, but we still want to follow the path's general trajectory.
     * @param pathName
     * 
     */
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

    /**
     * Returns the path to the closest front climbing position. This is an old version that essentially created a path on-the-fly;
     * it didn't work necessarily as intended and we ended up making a newer, more robust version.
     */
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

            // Dynamically selects which climb path to take based on the robot's current Y position.
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

    /**
     * Newer method that returns the path to the closest front climbing position. Depending on the robot's Y position on the field
     * it selects which post to climb on. Then it uses built-in PathPlanner pathfinding to intially path/move to the correct climbing path (left vs right).
     * Worked quite well but Mike's driving ended up being quicker most of the time...
     */
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
    
    /**
     * This method runs a path from the robot's frame of reference; therefore only the path's trajectory matters and not
     * its starting and ending poses.
     * @param pathName
     * 
     */
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

    /**
     * Old, deprecated method that works similarly to the other climb commands.
     * 
     */
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
    
    /**
     * Returns robot-relative chassis speeds. Used by PathPlanner
     * 
     */
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

    /**
     * Old unused method that gets robot pose from tags. Not used in 2026
     */
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

    /**
     * Old unused method that gets AprilTag pose and stores in swerve. Not used in 2026
     */
    public void getTargetPose(Pose2d targetPose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Blue){
            this.targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().plus(new Rotation2d(Math.PI)));
        }
        if (ally.get() == Alliance.Red){ 
            this.targetPose = targetPose; 
        }   

    }

    /**
     * Old unused method. Not used in 2026
     */
    public void resetFieldPoseWithTarget() {
        if (targetPose != null) {
            resetPose(targetPose);
        }
    }

    /**
     * Old unused method. Not used in 2026
     */
    public void resetLLPose() {
        if (poseLL != null) {
            resetPose(poseLL);
        }
    }

    /**
     * MegaTag2 is a feature/algorithm of the Limelight camera. Instead of just looking at 
     * AprilTags and guessing distance based on tag size (which can be noisy with one tag), it uses the robot's 
     * accurate Pigeon2 gyro compass data combined with the visual tag data to provide a rock-solid 
     * X/Y position coordinate, even while the robot is spinning wildly.
     * 
     * This method assumes CORRECT robot heading, and it doesn't calculate it as part of the algorithm -- unlike MT1.
     * At the end of the 2026 season we switched to the newer visionUpdate() method (which is similar).
     */

    public void MegaTag2UpdateOdometry() {

        // Step 1: Update the pose estimator with raw wheel movements.
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        boolean useMegaTag2 = true;

        if (useMegaTag2 == false)
        {
            // ... (Legacy MegaTag1 logic kept for fallback)
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

                // Feeds the robot's current precise rotation to the camera to assist its math.
                LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                
                if (estimateMT2 == null) {
                    break;
                }
                
                // Ignore vision data if we are spinning faster than 720 degrees per second (camera blur), 
                // or if no tags are visible.
                if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720 || estimateMT2.tagCount == 0)
                {
                    useThisEstimate = false;
                }

                if (useThisEstimate)
                {
                    // If the data is good, fuse it into the pose estimator to correct wheel drift.
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 999999));
                    m_poseEstimator.addVisionMeasurement(
                        estimateMT2.pose,
                        estimateMT2.timestampSeconds);
                }
            }
        }
    }

    /**
     * supplementary method used by visionUpdate() that tries to update pose estimator with MT1 (the original Limelight pose estimation algorithm). 
     * If it fails for any reason (such as high ambiguity), it returns false so that visionUpdate() can try using MT2 instead.
     * @param limelightName
     * @return true if MT1 data was successful, false otherwise
     */
    private boolean updateMT1(String limelightName) {
        LimelightHelpers.PoseEstimate estimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimateMT1 == null) {
            return false; // if estimate is null, skip to next limelight (this is to prevent errors/crashes when limelight is not detected for a split second, which happens sometimes when starting robot/building code)
        }

        if((estimateMT1.tagCount >= 2)) {   
            if(estimateMT1.rawFiducials[0].ambiguity > .7) { return false; }
            if(estimateMT1.rawFiducials[0].distToCamera > 3) { return false; }
        } else {
            return false; // less than 2 tags, no mt1
        }

        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,0.01));
        m_poseEstimator.addVisionMeasurement(
            estimateMT1.pose,
            estimateMT1.timestampSeconds);

        return true;
    }

    /**
     * A more versatile vision update method that first tries to use MegaTag1 to update the robot's odometry;
     * If there is high ambiguity or low trust in the MT1 data, it falls back to using MegaTag2, which is more robust to ambiguity but relies on accurate gyro (heading) data.
     * We created this method in 2026 after we found that the robot's pose (especially heading) was drifting significantly throughout the match, as an effort to combat that. 
     */
    public void visionUpdate() {
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        // boolean usedMegaTag1 = updateMT1(limelightName);

        for (String limelightName : Constants.Targeting.CAMERA_NAMES) {
            if (!updateMT1(limelightName)) { // initially tries to update with mt1, if failure will use mt2
                boolean useThisEstimate = true;
                // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)

                // line below is required because megatag2 requires the limelight to know the robot's current rotation, as it USES it instead of providing it like in MT1
                LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                
                // sometimes when starting robot/building code, mt2 == null for a split second, so need to check for that or code errors and crashes
                if (estimateMT2 == null) {
                    break;
                }
                
                if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720 || estimateMT2.tagCount == 0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
                {
                    useThisEstimate = false;
                }

                if (useThisEstimate)
                {
                    // n3 (yaw) set to high number because MegaTag2 uses robot's yaw instead of getting it for you
                    m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 999999)); // n3 was 9999999 
                    m_poseEstimator.addVisionMeasurement(
                        estimateMT2.pose,
                        estimateMT2.timestampSeconds);
                }
            }
        }
    }

    // --- VISION TARGETING & GAME MATH ---

    /**
     * This method calculates the PID output needed to rotate the robot to face the hub, based on the robot's current position and the known position of the hub.
     * The output is used by targeting commands to drive while having the robot aim simultaneously.
     * @param HUBX
     * @param HUBY
     * @return PID output
     */
    public double calculateTargetingPID (double HUBX, double HUBY) {
      Pose2d currentPose = getPose();

      double dx = HUBX - Units.metersToInches(currentPose.getX());
      double dy = HUBY - Units.metersToInches(currentPose.getY());
      double targetAngle = Math.atan2(dy, dx);

      double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
      return pidOutput;
    }

    /**
     * This method calculates the PID output needed to rotate the robot to face the correct alliance zone (red/blue)
     * Used by targeting commands for easily passing fuel from the Neutral Zone to our alliance zone.
     * @return PID output
     */
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

    /**
     * Similar to calculateTargetingPID but slightly different parameter inputs. Used in certain targeting commands.
     * @param targetAngle
     * @return PID output
     */
    public double calculateTargetingAutoPID(double targetAngle) {
        Pose2d currentPose = getPose();
        double pidOutput = pidControllerForTrackingOutput.calculate(currentPose.getRotation().getRadians(), targetAngle);
    
        return pidOutput;
    }

    /**
     * The goal of this method is to get the robot to rotate to face its direction of travel during path following. This is used
     * as a supplementary method for a command which can be used in PathPlanner GUI when assembling paths.
     * Did not work as intended and didn't see use in 2026 (was initially intended to help with ball collection)
     * 
     */
    public double getPPOverrideHeadingFeedback() {
        Pose2d currentPose = getPose();

        ChassisSpeeds robotChassisSpeeds = getChassisSpeeds();
        ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, getHeading());

        double targetAngle = new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond).getAngle().getRadians();
        SmartDashboard.putNumber("PPOverrideHeading target angle", targetAngle);

        SmartDashboard.putNumber("PPOverrideHeading PID output", PPHeadingPIDController.calculate(currentPose.getRotation().getRadians(), targetAngle));
        return PPHeadingPIDController.calculate(currentPose.getRotation().getRadians(), targetAngle);
    }

    /**
     * This method returns a command that when registered as a PathPlanner NamedCommand can be used in the GUI to override the rotation controller for that path.
     * The underlying rotation feedback for this method didn't work as intended. 
     * 
     */
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

    /**
     * This method returns a command that when registered as a PathPlanner NamedCommand can be used in the GUI to override the rotation controller for that path, 
     * but instead of facing the direction of travel, it faces the hub. Allows the robot to target the hub even while following PathPlannerAutos.
     */
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

     /**
     * "Shoot on the move" logic. If the robot is driving to the right, 
     * and shoots straight at the target, the game piece will miss to the right because it retains 
     * the robot's momentum. This method calculates a "Virtual Goal" by applying physics vectors, 
     * forcing the robot to aim slightly *behind* the real target to compensate for its own velocity.
     */

    public double[] getRotationMoving () {
        final double TIME_OF_FLIGHT = 1.2; // Estimated time the ball takes to land in the hub.

        ChassisSpeeds robotChassisSpeeds = getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, getHeading());

        Pose2d currentPose2d = getPose();

        double hubXMeters = Units.inchesToMeters(getHubX());
        double hubYMeters = Units.inchesToMeters(getHubY());

        // Offsets the goal based on current speed.
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

    // used by targeting methods for passing (pretty self-explanatory method name)
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

    /**
     * This method name is misleading but basically it is an all around aiming method used by targeting commands. If in the neutral zone, it will aim towards the alliance wall to help with passing. 
     * If outside the neutral zone, it will aim towards the hub while shaking the robot's heading slightly to agitate fuel. Also provides haptic feedback to the driver when 
     * they are aimed at the hub.
     * @param HUBX
     * @param HUBY
     * @param offsetRadians
     * @return PID output for targeting
     */
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

        // MENTOR NOTE: Triggers controller rumble (haptic feedback) to tell the driver they are aimed at the hub.
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

    /**
     * returns an offset to target to that changes sinusoidally over time to agitate fuel.
     * 
     */
    public double getShakingOffset() {
        double shakeFrequency = Constants.Targeting.SHAKE_FREQUENCY;
        double shakeSpread = Constants.Targeting.SHAKE_SPREAD;

        double distanceInches = Math.max(getDistanceToHub(), 1.0);

        double amplitudeRadians = shakeSpread / distanceInches;

        double time = Timer.getFPGATimestamp();
        return amplitudeRadians * Math.sin(2 * Math.PI * shakeFrequency * time);
    }

    /**
     * method which is now unused which returns whether or not the robot is obstructed by the hubs, or in other words in the vertical center of the field.
     * @param yPose
     * 
     */
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
        // update robot odometry every 20ms using vision (if available)
        visionUpdate();
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