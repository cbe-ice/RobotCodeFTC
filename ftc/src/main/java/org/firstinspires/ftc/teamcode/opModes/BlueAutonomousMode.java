package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ShootingMechanisms;

@Autonomous(name = "Blue Auto Mode", group = "Auto")
public class BlueAutonomousMode extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private final ShootingMechanisms shootingMechanisms = new ShootingMechanisms();
    boolean row1, row2, row3;
    private boolean shotsTriggered = false;

    public enum PathState {
        DRIVE_START_SHOOT,
        SHOOT,
        ROW_1,
        ROW_2,
        ROW_3,
        FINISH
    }

    PathState pathState;
    final Pose startPose = new Pose(20.5, 121.5, Math.toRadians(143.5));
    final Pose shootPose = new Pose(40, 102, Math.toRadians(130));
    final Pose startRow1 = new Pose(48, 84, Math.toRadians(180));
    final Pose endRow1 = new Pose(16, 84, Math.toRadians(180));
    final Pose startRow2 = new Pose(48, 60, Math.toRadians(180));
    final Pose endRow2 = new Pose(9, 60, Math.toRadians(180));
    final Pose exitRow2 = new Pose(24, 60, Math.toRadians(180));
    final Pose startRow3 = new Pose(48, 36, Math.toRadians(180));
    final Pose endRow3 = new Pose(9, 36, Math.toRadians(180));
    final Pose endPose = new Pose(34.5, 93.5, Math.toRadians(136.5));

    private PathChain startToShoot,
            shootToRow1, collectRow1, row1ToShoot,
            shootToRow2, collectRow2, endRow2ToExitRow2, row2ToShoot,
            shootToRow3, collectRow3, row3ToShoot,
            shootToEnd;

    public void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToRow1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startRow1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startRow1.getHeading())
                .build();
        collectRow1 = follower.pathBuilder()
                .addPath(new BezierLine(startRow1, endRow1))
                .setLinearHeadingInterpolation(startRow1.getHeading(), endRow1.getHeading())
                .build();
        row1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(endRow1, shootPose))
                .setLinearHeadingInterpolation(endRow1.getHeading(), shootPose.getHeading())
                .build();
        shootToRow2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startRow2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startRow2.getHeading())
                .build();
        collectRow2 = follower.pathBuilder()
                .addPath(new BezierLine(startRow2, endRow2))
                .setLinearHeadingInterpolation(startRow2.getHeading(), endRow2.getHeading())
                .build();
        endRow2ToExitRow2 = follower.pathBuilder()
                .addPath(new BezierLine(endRow2, exitRow2))
                .setLinearHeadingInterpolation(endRow2.getHeading(), exitRow2.getHeading())
                .build();

        row2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(endRow2, shootPose))
                .setLinearHeadingInterpolation(endRow2.getHeading(), shootPose.getHeading())
                .build();
        shootToRow3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, startRow3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), startRow3.getHeading())
                .build();
        collectRow3 = follower.pathBuilder()
                .addPath(new BezierLine(startRow3, endRow3))
                .setLinearHeadingInterpolation(startRow3.getHeading(), endRow3.getHeading())
                .build();
        row3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(endRow3, shootPose))
                .setLinearHeadingInterpolation(endRow3.getHeading(), shootPose.getHeading())
                .build();
        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_SHOOT:
                follower.followPath(startToShoot, true);
                setPathState(PathState.SHOOT);
                break;

            case SHOOT:
                if(!follower.isBusy()) {
                    if(!shotsTriggered) {
                        shootingMechanisms.autoShots(3);
                        shotsTriggered = true;
                    } else if (shotsTriggered && !shootingMechanisms.isBusy()) {
                        if (!row1) {
                            setPathState(PathState.ROW_1);
                        } else if (!row2) {
                            setPathState(PathState.ROW_2);
                        } else if (!row3) {
                            setPathState(PathState.ROW_3);
                        } else {
                            setPathState(PathState.FINISH);
                        }
                    }
                }
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {

                }

                break;
            case ROW_1:
                follower.followPath(shootToRow1, true);
                follower.followPath(collectRow1, true);
                follower.followPath(row1ToShoot, true);
                setPathState(PathState.SHOOT);
                row1 = true;
                break;

            case ROW_2:
                follower.followPath(shootToRow2, true);
                follower.followPath(collectRow2, true);
                follower.followPath(endRow2ToExitRow2, true);
                follower.followPath(row2ToShoot, true);
                setPathState(PathState.SHOOT);
                row2 = true;
                break;

            case ROW_3:
                follower.followPath(shootToRow3, true);
                follower.followPath(collectRow3, true);
                follower.followPath(row3ToShoot, true);
                setPathState(PathState.SHOOT);
                row3 = true;
                break;

            case FINISH:
                follower.followPath(shootToEnd, true);
                break;
            default:
                break;
        }

    }

    public void setPathState(PathState newState)    {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;

    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = TwoWheelLocalizer.createFollower(hardwareMap);

        shootingMechanisms.init(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        row1 = false;
        row2 = false;
        row3 = false;
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);


    }

    @Override
    public void loop() {
        follower.update();
        shootingMechanisms.update();
        statePathUpdate();

        telemetry.addData("Path State ", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());

    }
}