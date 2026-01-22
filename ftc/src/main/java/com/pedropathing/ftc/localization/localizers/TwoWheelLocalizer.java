package com.pedropathing.ftc.localization.localizers;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TwoWheelLocalizer {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.35)
            .forwardZeroPowerAcceleration(-32.69149444627547)
            .lateralZeroPowerAcceleration(-25.8163065750978)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.001, 0.0425, 0, 0.09))
            .headingPIDFCoefficients(new PIDFCoefficients(0.001, 0.045, 0, 0.9))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0025, 0.025, 0, 0.25, 0.6))
            .centripetalScaling(0.000125);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontMotor")
            .rightRearMotorName("rightBackMotor")
            .leftRearMotorName("leftBackMotor")
            .leftFrontMotorName("leftFrontMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(25.961392078908045)
            .yVelocity(22.883289391366258);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(4.75)
            .strafePodX(6.5)
            .forwardEncoder_HardwareMapName("rightFrontMotor")
            .strafeEncoder_HardwareMapName("leftBackMotor")
            . IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.95, 2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
