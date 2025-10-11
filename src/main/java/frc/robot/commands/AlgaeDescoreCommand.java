package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.BallGrabber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Utility.AutoAlignPath;
import frc.robot.Utility.Setpoint;
import frc.robot.Utility.Enums.DereefIndex;

import frc.robot.Utility.Enums.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A command that automatically aligns the robot to a target pose, actuates the elevator and wrist, scores, and retracts.
 * This command is used for descoring algae at the reef on any level
 */
public class AlgaeDescoreCommand extends SequentialCommandGroup {

    // Poses
    private Pose2d targetPose;

    // Paths
    private AutoAlignPath path;

    // Profiles
    private TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // The position is actually the error.
    private TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.Constraints translationConstraints;
    private TrapezoidProfile.Constraints rotationConstraint;
    private TrapezoidProfile translationProfile;
    private TrapezoidProfile rotationProfile;

    // Subsystems
    private SwerveSubsystem drivetrain;
    private Arm arm;
    private Elevator elevator;
    private BallGrabber grabber;

    // Locations
    DereefIndex location;
    ReefLevel level;

    // Targets
    private double elevatorTarget;
    private double wristTarget;


    /** Constructor for our scoring command */
    public AlgaeDescoreCommand(Arm arm, Elevator elevator, BallGrabber grabber,SwerveSubsystem drivetrain,
    DereefIndex location, ReefLevel level) {

        this.drivetrain = drivetrain;
        this.arm = arm;
        this.elevator = elevator;
        this.grabber = grabber;

        this.location = location;
        this.level = level;

        setTargets();
        
        
        super.addCommands(
            getDriveInCommand(),
            getActuateCommand(),
            getIntakeAlgae(),
            getDeactuationCommand()
       );
    

        super.addRequirements(arm, elevator, grabber, drivetrain);
    }

    /** Gets the algae intake command */
    public Command getIntakeAlgae() {
        return new RunCommand(() -> grabber.runIntake(), grabber)
                .until(() -> grabber.getOutputCurrent() > 25.0) 
                .withTimeout(3.0); // Safety timeout
    }

    /** Helper method for following a straight trajectory with a trapezoidal profile */
    public Command getFollowPath(AutoAlignPath path, double finalVelocity){
        final Command followPath = new FunctionalCommand(
            () -> {
            // Resets profiles and states
            translationConstraints = new Constraints(path.maxVelocity, path.maxAcceleration);
            rotationConstraint = new Constraints(path.maxAngularVelocity, path.maxAngularAcceleration);

            translationProfile = new TrapezoidProfile(translationConstraints);
            rotationProfile = new TrapezoidProfile(rotationConstraint);

            translationalState.position = path.getDisplacement().getNorm();
            translationalState.velocity = MathUtil.clamp(((-1) * (drivetrain.getFieldVelocity().vxMetersPerSecond * path.getDisplacement().getX() 
                                                                + drivetrain.getFieldVelocity().vyMetersPerSecond * path.getDisplacement().getY())/ translationalState.position),
                                                                -path.maxVelocity,
                                                                0);

            targetTranslationalState.velocity = finalVelocity;

            targetRotationalState.position = path.endPose.getRotation().getRadians();
            rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
            rotationalState.velocity = drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond;
        },
            () -> {
                // Move.
                Setpoint newSetpoint = getNextSetpoint(path);
                drivetrain.followSegment(newSetpoint, targetPose);
                if(drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.02
                    && Math.abs(drivetrain.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.017
                ){
                    // Stop.
                    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
                }
            },
            (interrupted) -> {},
            () -> drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.02
            && Math.abs(drivetrain.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.017
        );
        return followPath;

    }

    /** Drives close to our target pose during auto */
    public Command getDriveToPrescore(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            getPrescore(targetPose),
            DrivebaseConstants.kMaxAutoVelocity,
            DrivebaseConstants.kMaxAutoAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveToPrescore = getFollowPath(path, 2.1).until(
            () -> drivetrain.getPose().getTranslation().getDistance(getPrescore(targetPose).getTranslation()) < 0.2
        );
        return driveToPrescore;
    }

    /** Snaps to the reef pose */
    public Command getDriveInCommand(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            targetPose, //originally just target pose
            DrivebaseConstants.kMaxAlignmentVelocity,
            DrivebaseConstants.kMaxAlignmentAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveIn = getFollowPath(path, 0);
        return driveIn;
    }

    /** Actuates superstructure to our desired level */
    /** Actuates superstructure to our desired level for algae collection */
    /** Actuates superstructure to our desired level for algae collection */
    public Command getActuateCommand(){
        return new ParallelCommandGroup(
            new RunCommand(() -> elevator.setElevatorPosition(elevatorTarget), elevator)
                .until(() -> Math.abs(elevator.getElevatorHeight() - elevatorTarget) < ElevatorConstants.kElevatorTolerance),
            new RunCommand(() -> wrist.setMotorPosition(wristTarget), wrist)
                .until(() -> Math.abs(elevator.getElevatorHeight() - elevatorTarget) < ElevatorConstants.kElevatorTolerance)
        );
    }

    /** Deactuates the superstructure in teleop for driver QOL */
    public Command getDeactuationCommand(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            get_backpuPose2d(targetPose),
            DrivebaseConstants.kMaxAlignmentVelocity, 
            DrivebaseConstants.kMaxAlignmentAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command deactuateSuperstructure = new FunctionalCommand(
            () -> {


            },
            () -> { 
                // Retract superstructure to stow position
                wrist.setMotorPosition(WristConstants.WristStates.S_L2);
                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
            },
            (interrupted) -> {
                // Ensure grabber is stopped if interrupted
                grabber.stop();
            },
            () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < ElevatorConstants.kElevatorTolerance
                  && Math.abs(wrist.getWristPosition() - WristConstants.WristStates.S_L2) < 5.0)
        );
        final Command backUp = getFollowPath(path, 0);
        return Commands.sequence(backUp, deactuateSuperstructure).withTimeout(3);
    }

    /** Gets the prescore for a specific Pose2d */
    public Pose2d getPrescore(Pose2d targetPose){
        return new Pose2d(
            targetPose.getX() + 1 * Math.cos(targetPose.getRotation().getRadians()), 
            targetPose.getY() + 1 * Math.sin(targetPose.getRotation().getRadians()), 
            targetPose.getRotation()
        );
    }

    public Pose2d get_backpuPose2d(Pose2d targetPose){
        return new Pose2d(
            targetPose.getX() + 0.5 * Math.cos(targetPose.getRotation().getRadians()), 
            targetPose.getY() + 0.5 * Math.sin(targetPose.getRotation().getRadians()), 
            targetPose.getRotation()
        );
    }

    /** Sets the target for the robot, including target pose, elevator height, and arm angle */
    private void setTargets() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        final double DISTANCE_FROM_TAG = 0.164;

        HashMap<DereefIndex, Pose2d> targetPoses = new HashMap<>();
        targetPoses.put(DereefIndex.BOTTOM_RIGHT, new Pose2d(13.376, 2.66, Rotation2d.fromDegrees(300)));
        targetPoses.put(DereefIndex.FAR_RIGHT, new Pose2d(14.412, 3.613, Rotation2d.fromDegrees(0)));
        targetPoses.put(DereefIndex.TOP_RIGHT, new Pose2d(14.083, 4.978, Rotation2d.fromDegrees(60)));
        targetPoses.put(DereefIndex.TOP_LEFT, new Pose2d(12.868, 5.368, Rotation2d.fromDegrees(120)));
        targetPoses.put(DereefIndex.FAR_LEFT, new Pose2d(11.718, 4.429, Rotation2d.fromDegrees(180)));
        targetPoses.put(DereefIndex.BOTTOM_LEFT, new Pose2d(12.06, 3.075, Rotation2d.fromDegrees(240)));

        Pose2d tagPose = targetPoses.get(location);

        targetPose = new Pose2d(
            tagPose.getX() + DISTANCE_FROM_TAG * Math.cos(tagPose.getRotation().getRadians() + Math.PI/2),
            tagPose.getY() + DISTANCE_FROM_TAG * Math.sin(tagPose.getRotation().getRadians() + Math.PI/2),
            tagPose.getRotation()
        );

        if(alliance == DriverStation.Alliance.Blue){
            targetPose = applyAllianceTransform(targetPose);
        }

        switch(level){
            case LOW_BALL:
                elevatorTarget = FieldConstants.Elevator_low_algae;
                wristTarget = FieldConstants.Wrist_low_algae;
                break;
            case HIGH_BALL:
                elevatorTarget = FieldConstants.Elevator_top_algae;
                wristTarget = FieldConstants.Wrist_top_algae;
                break;
            default:
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                wristTarget = WristConstants.WristStates.STOW;
        }
    }

    /** Transforms red alliance poses to blue by reflecting around the center point of the field*/
    public Pose2d applyAllianceTransform(Pose2d pose){
        return new Pose2d(
            pose.getX() - 2 * (pose.getX() - 8.75),
            pose.getY() - 2 * (pose.getY() - 4.0),
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );
    }

    /** Gets a setpoint for the robot PID to follow.
     * @return A Setpoint object representing the next target robot state on a certain auto align path.
     */
    public Setpoint getNextSetpoint(AutoAlignPath path){
        // trans
        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
        translationalState.position = translationSetpoint.position;
        translationalState.velocity = translationSetpoint.velocity;

        // rot wraparound calculations
        double rotationalPose = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
        double goalRotation = targetRotationalState.position;
        double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);

        targetRotationalState.position = rotationalPose + angularError;
        double setpointError = MathUtil.angleModulus(rotationalState.position - rotationalPose);
        rotationalState.position = rotationalPose + setpointError;
        // rot
        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
        rotationalState.position = rotationalSetpoint.position;
        rotationalState.velocity = rotationalSetpoint.velocity;

        // arc length parametrization for a line
        Translation2d interpolatedTranslation = path.endPose
            .getTranslation()
            .interpolate(path.startPose.getTranslation(), 
            translationSetpoint.position / path.getDisplacement().getNorm());

        return new Setpoint(
            interpolatedTranslation.getX(),
            interpolatedTranslation.getY(),
            rotationalState.position,
            path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, 
            path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
            rotationalState.velocity
        );
    }
}
