package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "PIDtestcomments", group = "Linear Opmode")
public class PIDtestcomments extends LinearOpMode {

    // Drive motors
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    // Flywheel shooter motors
    private DcMotorEx flywheelBig = null;
    private DcMotorEx flywheelSmall = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Driving behavior settings
    private boolean inverted = false;   // Allows flipping driving direction when needed
    private double wheelScale = 1.0;    // Speed scaling (e.g. slow mode)

    // Shooter state machine modes
    private enum ShooterState { IDLE, SPINUP, SHOOT, COOLDOWN }
    private ShooterState shooterState = ShooterState.IDLE;

    // Desired flywheel RPM
    private double targetRPM = 0.0;

    // PIDF controller for flywheel speed control
    private PIDF flywheelPid = new PIDF(0.001, 0.0, 0.0005, 0.0);

    @Override
    public void runOpMode() {
        // Map motors from configuration
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        flywheelBig = hardwareMap.get(DcMotorEx.class, "Flywheel_Big");
        flywheelSmall = hardwareMap.get(DcMotorEx.class, "Flywheel_Small");

        // Set drive motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Flywheel direction (depends on mechanical layout)
        flywheelBig.setDirection(DcMotor.Direction.FORWARD);
        flywheelSmall.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ----- DRIVE CONTROL -----
            // Standard tank/arcade blend: left stick = drive, right stick = turn
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            double leftPower  = (drive + turn) * wheelScale;
            double rightPower = (drive - turn) * wheelScale;

            // If driving is inverted, reverse motor output
            if (!inverted) {
                leftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
                rightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
            } else {
                leftDrive.setPower(Range.clip(-leftPower, -1.0, 1.0));
                rightDrive.setPower(Range.clip(-rightPower, -1.0, 1.0));
            }

            // ----- SHOOTER CONTROL -----
            // A button → start spin-up sequence
            if (gamepad1.a) {
                targetRPM = 3300.0; // Target flywheel speed
                if (shooterState == ShooterState.IDLE)
                    shooterState = ShooterState.SPINUP;

            // X button → stop shooter and reset to idle
            } else if (gamepad1.x) {
                targetRPM = 0.0;
                shooterState = ShooterState.IDLE;
            }

            // B button → force shot if flywheel is spun up
            if (gamepad1.b && shooterState == ShooterState.SPINUP) {
                shooterState = ShooterState.SHOOT;
            }

            // Run the shooter logic
            runShooterStateMachine();

            // Telemetry output
            telemetry.addData("Inverted", inverted);
            telemetry.addData("State", shooterState);
            telemetry.addData("TargetRPM", targetRPM);
            telemetry.addData("FlywheelPowerBig", flywheelBig.getPower());
            telemetry.addData("FlywheelPowerSmall", flywheelSmall.getPower());
            telemetry.update();
        }
    }

    /**
     * Shooter finite state machine.
     * Controls spin-up, firing, and cool-down behavior.
     */
    private void runShooterStateMachine() {

        // Read current flywheel RPM (or default to 0 if unsupported)
        double currentRPM = 0.0;
        try {
            currentRPM = Math.abs(flywheelBig.getVelocity());
        } catch (Exception ignored) {
            currentRPM = 0.0;
        }

        switch (shooterState) {
            case IDLE:
                // Motors off when idle
                flywheelBig.setPower(0.0);
                flywheelSmall.setPower(0.0);
                break;

            case SPINUP:
                // Apply PIDF to reach target RPM
                double powerCommand = flywheelPid.update(targetRPM, currentRPM);
                powerCommand = Range.clip(powerCommand, 0.0, 1.0);

                flywheelBig.setPower(powerCommand);     // Main flywheel
                flywheelSmall.setPower(0.0);            // Intake off during spin-up

                // If within tolerance, auto-transition to SHOOT
                if (Math.abs(currentRPM - targetRPM) < 150.0 && targetRPM > 0) {
                    shooterState = ShooterState.SHOOT;
                }
                break;

            case SHOOT:
                // Run the small wheel to feed a ring
                double intakePower = 1.0;
                flywheelSmall.setPower(intakePower);

                // Short delay for firing
                sleep(150);

                // Stop intake
                flywheelSmall.setPower(0.0);

                // Move to cooldown
                shooterState = ShooterState.COOLDOWN;
                break;

            case COOLDOWN:
                // Pause briefly to stabilize shooter state
                sleep(150);

                // If still firing, go back to spin-up
                shooterState = (targetRPM > 0) ? ShooterState.SPINUP : ShooterState.IDLE;
                break;
        }
    }

    /**
     * Basic PIDF controller for motor velocity control.
     */
    public static class PIDF {
        private final double kP, kI, kD, kF;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double lastTime = System.nanoTime() / 1e9;

        public PIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public double update(double target, double actual) {
            double now = System.nanoTime() / 1e9;
            double dt = Math.max(1e-6, now - lastTime);
            lastTime = now;

            // Compute standard PIDF terms
            double error = target - actual;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            // Return total motor power command
            return (kP * error)
                 + (kI * integral)
                 + (kD * derivative)
                 + (kF * target);
        }

        public void reset() {
            integral = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime() / 1e9;
        }
    }

}
