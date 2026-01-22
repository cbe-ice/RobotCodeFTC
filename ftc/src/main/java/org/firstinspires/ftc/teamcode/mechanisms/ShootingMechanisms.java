package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Subsystem for the robot's intake mechanism.
 * Controls the intake motor and the loading motor.
 */
public class ShootingMechanisms {

    private DcMotor intakeMotor, loadMotor, leftFlywheel, rightFlywheel;
    private ElapsedTime stateTimer = new ElapsedTime();

    // ===================== CONFIGURATION =====================

    private static final double shotRPM = 0.45;
    private static final double shotDuration = 1;
    private static final double shotDelay = 0.5;
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double maxSpinTime = 2;

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET

    }

    private FlywheelState flywheelState;

    /**
     * Initializes the intake hardware.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        loadMotor = hwMap.get(DcMotor.class, "loadMotor");
        leftFlywheel = hwMap.get(DcMotor.class, "leftFlyWheel");
        rightFlywheel = hwMap.get(DcMotor.class, "rightFlyWheel");

        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelState = FlywheelState.IDLE;
    }

    public void update() {
        intakeMotor.setPower(1);

        switch(flywheelState){
            case IDLE:
                if(shotsRemaining > 0) {
                    loadMotor.setPower(0.05);
                    leftFlywheel.setPower(shotRPM);
                    rightFlywheel.setPower(shotRPM);

                    stateTimer.reset();

                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if(flywheelVelocity > shotRPM || stateTimer.seconds() > maxSpinTime) {
                    loadMotor.setPower(1);
                    stateTimer.reset();

                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if(stateTimer.seconds() > shotDuration) {
                    shotsRemaining--;
                    loadMotor.setPower(0.05);
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET;
                }
                break;
            case RESET:
                if(stateTimer.seconds() > shotDelay) {
                    if(shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        leftFlywheel.setPower(0);
                        rightFlywheel.setPower(0);
                        flywheelState = FlywheelState.IDLE;
                    }

                }
                break;
        }

    }

    public void autoShots(int numShots) {
        if(flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numShots;
        }

    }

    public boolean isBusy() {
        return flywheelState !=FlywheelState.IDLE;
    }

    /**
     * Sets the power of the intake motor.
     *
     * @param intakeSpeed Power level (-1.0 to 1.0)
     */
    public void intake(double intakeSpeed) {
        intakeMotor.setPower(intakeSpeed);
    }

    /**
     * Controls the loading mechanism with a pulsed action.
     * Uses real-time based timing for consistent behavior regardless of loop
     * frequency.
     *
     * @param loadSpeed  Power level for the loader
     */
    public void load(double loadSpeed ) {
        loadMotor.setPower(loadSpeed);
    }

    /**
     * Sets the power for both flywheels.
     *
     * @param shootSpeed Power level (0.0 to 1.0 usually)
     */
    public void shoot(double shootSpeed) {
        leftFlywheel.setPower(shootSpeed);
        rightFlywheel.setPower(shootSpeed);

    }
    /**
     * Stops all intake motors immediately.
     * Resets the loader timer state.
     */
    public void stopAll() {
        intakeMotor.setPower(0);
        loadMotor.setPower(0);
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }
}
