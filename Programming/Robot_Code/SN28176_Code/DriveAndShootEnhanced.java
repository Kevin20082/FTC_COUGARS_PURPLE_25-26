package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriveAndShoot-Enhanced", group = "Linear Opmode")
public class DriveAndShootEnhanced extends LinearOpMode {

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

    // PIDF controller for flywheel speed control (initial gains)
    private PIDF flywheelPid = new PIDF(0.001, 0.0, 0.0005, 0.0);

    // --- Automatic multi-shot variables ---
    private boolean autoMultiShotEnabled = false;   // Toggleable automatic multi-shot
    private long lastFeedTime = 0L;                // ms timestamp of last feed
    private long feedIntervalMs = 500L;            // time between auto feeds when at speed
    private long feedDurationMs = 150L;            // how long to run the feed motor per shot
    private boolean feeding = false;               // currently feeding (used by non-blocking feed)
    private long feedStartTime = 0L;               // when the current feed started

    // --- PID tuning helpers ---
    private int pidSelect = 0;   // 0=kP,1=kI,2=kD,3=kF
    // Step sizes for tuning (adjustable)
    private final double pidSmallStep = 0.0001;
    private final double pidLargeStep = 0.001;

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

        // Used to detect button edge presses
        boolean prevRightBumper = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;

        boolean prevGamepad2X = false;
        boolean prevGamepad2Y = false;
        boolean prevGamepad2A = false;
        boolean prevGamepad2B = false;

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

            // ----- DRIVER RPM ADJUSTMENTS (gamepad1) -----
            // Fine adjust: dpad_up/dpad_down
            if (gamepad1.dpad_up && !prevDpadUp) {
                targetRPM += 50.0;
            } else if (gamepad1.dpad_down && !prevDpadDown) {
                targetRPM = Math.max(0.0, targetRPM - 50.0);
            }
            // Coarse adjust: dpad_right/dpad_left
            if (gamepad1.dpad_right && !prevDpadRight) {
                targetRPM += 500.0;
            } else if (gamepad1.dpad_left && !prevDpadLeft) {
                targetRPM = Math.max(0.0, targetRPM - 500.0);
            }

            // Store previous dpad states for edge detection
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;

            // Toggle auto multi-shot mode with right bumper (edge triggered)
            if (gamepad1.right_bumper && !prevRightBumper) {
                autoMultiShotEnabled = !autoMultiShotEnabled;
                // Reset feed timers so auto-fire starts cleanly
                lastFeedTime = System.currentTimeMillis();
                feeding = false;
            }
            prevRightBumper = gamepad1.right_bumper;

            // ----- SHOOTER CONTROL (existing triggers remain) -----
            // A button → start spin-up sequence (existing)
            if (gamepad1.a) {
                targetRPM = 3300.0; // Target flywheel speed
                if (shooterState == ShooterState.IDLE) shooterState = ShooterState.SPINUP;
                // reset PID integral when we set a large new target
                flywheelPid.onTargetChange(targetRPM);
            } else if (gamepad1.x) {
                // X button → stop shooter and reset to idle (existing)
                targetRPM = 0.0;
                shooterState = ShooterState.IDLE;
                flywheelPid.onTargetChange(targetRPM);
            }

            // B button → manual single shot trigger (existing)
            if (gamepad1.b && shooterState == ShooterState.SPINUP) {
                // If you want single-shot while at spinup, set shoot state
                shooterState = ShooterState.SHOOT;
            }

            // ----- PID TUNING (gamepad2) -----
            // gamepad2.x cycles selected param (edge)
            if (gamepad2.x && !prevGamepad2X) {
                pidSelect = (pidSelect + 1) % 4;
            }
            // gamepad2.y increases selected gain
            if (gamepad2.y && !prevGamepad2Y) {
                adjustSelectedPID(true, gamepad2.left_bumper); // hold left_bumper for bigger step
            }
            // gamepad2.a decreases selected gain
            if (gamepad2.a && !prevGamepad2A) {
                adjustSelectedPID(false, gamepad2.left_bumper);
            }
            // gamepad2.b resets PID integral & derivative (edge)
            if (gamepad2.b && !prevGamepad2B) {
                flywheelPid.reset();
            }
            prevGamepad2X = gamepad2.x;
            prevGamepad2Y = gamepad2.y;
            prevGamepad2A = gamepad2.a;
            prevGamepad2B = gamepad2.b;

            // ----- Run shooter state machine (non-blocking auto-shoot integrated) -----
            runShooterStateMachineNonBlocking();

            // ----- Telemetry -----
            telemetry.addData("Inverted", inverted);
            telemetry.addData("State", shooterState);
            telemetry.addData("TargetRPM", "%.0f", targetRPM);
            telemetry.addData("CurrentRPM", "%.0f", getCurrentRPM());
            telemetry.addData("FlywheelPowerBig", "%.2f", flywheelBig.getPower());
            telemetry.addData("FlywheelPowerSmall", "%.2f", flywheelSmall.getPower());
            telemetry.addData("AutoMultiShot", autoMultiShotEnabled);
            telemetry.addData("FeedInterval(ms)", feedIntervalMs);
            telemetry.addData("FeedDuration(ms)", feedDurationMs);
            telemetry.addData("PID Selected", pidName(pidSelect));
            telemetry.addData("kP", "%.6f", flywheelPid.getKP());
            telemetry.addData("kI", "%.6f", flywheelPid.getKI());
            telemetry.addData("kD", "%.6f", flywheelPid.getKD());
            telemetry.addData("kF", "%.6f", flywheelPid.getKF());
            telemetry.update();
        }
    }

    /**
     * Non-blocking shooter FSM that supports automatic multi-shot mode.
     * Reworked so shoot/cooldown don't sleep for long durations inside the loop.
     */
    private void runShooterStateMachineNonBlocking() {

        double currentRPM = getCurrentRPM();

        // 1) Primary PID spin-up (if needed)
        if (shooterState == ShooterState.SPINUP) {
            double powerCommand = flywheelPid.update(targetRPM, currentRPM);
            powerCommand = Range.clip(powerCommand, 0.0, 1.0);
            flywheelBig.setPower(powerCommand);
            // keep small wheel off until we actually feed/shoot
            if (!feeding) flywheelSmall.setPower(0.0);

            // If within tolerance, move to SHOOT state or stay in SPINUP based on auto flag
            if (Math.abs(currentRPM - targetRPM) < 150.0 && targetRPM > 0.0) {
                // if auto multi-shot enabled, start auto-shot mode (stay in SPINUP but manage feeding)
                if (autoMultiShotEnabled) {
                    // we keep shooterState as SPINUP but handle automatic feeding below
                    // mark lastFeedTime so auto will start promptly
                    if (lastFeedTime == 0L) lastFeedTime = System.currentTimeMillis();
                } else {
                    // If auto not enabled, go to SHOOT only when manual trigger (B) is pressed,
                    // but we still allow a single-shot transition here if desired.
                    // To preserve existing behavior that auto-transitions to SHOOT, do:
                    shooterState = ShooterState.SHOOT;
                }
            }
        } else if (shooterState == ShooterState.IDLE) {
            // Ensure motors are stopped
            flywheelBig.setPower(0.0);
            flywheelSmall.setPower(0.0);
        } else if (shooterState == ShooterState.SHOOT) {
            // Manual single-shot: run feeder for feedDurationMs but non-blocking.
            if (!feeding) {
                feeding = true;
                feedStartTime = System.currentTimeMillis();
                flywheelSmall.setPower(1.0);
            }
            // Manage feeding timeout
            if (feeding) {
                if (System.currentTimeMillis() - feedStartTime >= feedDurationMs) {
                    // stop feeding
                    flywheelSmall.setPower(0.0);
                    feeding = false;
                    // move to cooldown
                    shooterState = ShooterState.COOLDOWN;
                    // set lastFeedTime so auto won't immediately feed again
                    lastFeedTime = System.currentTimeMillis();
                }
            }
        } else if (shooterState == ShooterState.COOLDOWN) {
            // Short cooldown implemented non-blocking - use timestamp
            if (!feeding) {
                feeding = true;
                feedStartTime = System.currentTimeMillis();
            }
            // We'll use feedStartTime as cooldown start for reuse
            if (System.currentTimeMillis() - feedStartTime >= feedDurationMs) {
                feeding = false;
                // If targetRPM still > 0, go back to SPINUP (which will maintain speed)
                shooterState = (targetRPM > 0.0) ? ShooterState.SPINUP : ShooterState.IDLE;
            }
        }

        // ----- Automatic multi-shot handling (independent of above manual states) -----
        // If autoMultiShotEnabled and flywheel is at speed, periodically run the feeder automatically.
        // This does not require entering SHOOT state; it will feed while keeping flywheelBig powered by PID.
        if (autoMultiShotEnabled && targetRPM > 0.0 && Math.abs(currentRPM - targetRPM) < 150.0) {
            long now = System.currentTimeMillis();

            // if not currently feeding and enough time elapsed since last feed, start feeding
            if (!feeding && (now - lastFeedTime >= feedIntervalMs)) {
                feeding = true;
                feedStartTime = now;
                flywheelSmall.setPower(1.0);
            }

            // if currently feeding, stop after feedDurationMs
            if (feeding && (now - feedStartTime >= feedDurationMs)) {
                feeding = false;
                flywheelSmall.setPower(0.0);
                lastFeedTime = now;
                // continue with spinup state (leave flywheel running)
                shooterState = ShooterState.SPINUP;
            }
        }
    }

    /**
     * Small helper: reads current flywheel RPM while handling exceptions.
     */
    private double getCurrentRPM() {
        double currentRPM = 0.0;
        try {
            currentRPM = Math.abs(flywheelBig.getVelocity());
        } catch (Exception ignored) {
            currentRPM = 0.0;
        }
        return currentRPM;
    }

    /**
     * Adjust selected PID gain from gamepad2 commands.
     * If 'bigStep' is true (hold left bumper on gamepad2), use larger step.
     */
    private void adjustSelectedPID(boolean increase, boolean bigStep) {
        double step = bigStep ? pidLargeStep : pidSmallStep;
        double signed = increase ? step : -step;
        switch (pidSelect) {
            case 0:
                flywheelPid.setKP(Math.max(0.0, flywheelPid.getKP() + signed));
                break;
            case 1:
                flywheelPid.setKI(Math.max(0.0, flywheelPid.getKI() + signed));
                break;
            case 2:
                flywheelPid.setKD(Math.max(0.0, flywheelPid.getKD() + signed));
                break;
            case 3:
                flywheelPid.setKF(Math.max(0.0, flywheelPid.getKF() + signed));
                break;
        }
    }

    private String pidName(int idx) {
        switch (idx) {
            case 0: return "kP";
            case 1: return "kI";
            case 2: return "kD";
            case 3: return "kF";
            default: return "?";
        }
    }

    /**
     * Basic PIDF controller for motor velocity control.
     * Enhanced:
     *  - simple anti-windup: integral clamped to a max magnitude based on target
     *  - reset integral/derivative on large target changes (call onTargetChange)
     *  - getters/setters to allow runtime tuning
     */
    public static class PIDF {
        private double kP, kI, kD, kF;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double lastTime = System.nanoTime() / 1e9;

        // anti-windup clamp value (tunable internally)
        private double integralClamp = 1000.0;

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

            double error = target - actual;

            // anti-windup: clamp integral to avoid runaway
            integral += error * dt;
            integral = clamp(integral, -integralClamp, integralClamp);

            double derivative = (error - lastError) / dt;
            lastError = error;

            // compute and return
            return (kP * error) + (kI * integral) + (kD * derivative) + (kF * target);
        }

        /**
         * Call this when the target RPM changes significantly so we reset integral/derivative history.
         */
        public void onTargetChange(double newTarget) {
            integral = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime() / 1e9;
        }

        public void reset() {
            integral = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime() / 1e9;
        }

        private double clamp(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        // Getters / setters for runtime tuning
        public double getKP() { return kP; }
        public double getKI() { return kI; }
        public double getKD() { return kD; }
        public double getKF() { return kF; }
        public void setKP(double v) { this.kP = v; }
        public void setKI(double v) { this.kI = v; }
        public void setKD(double v) { this.kD = v; }
        public void setKF(double v) { this.kF = v; }
    }
}

/**
 * DriveAndShoot (enhanced)
 *
 * Changes from previous version:
 *  - Improved PIDF (anti-windup, integral reset on target change).
 *  - Added automatic multi-shot mode (toggleable) that repeatedly feeds when at speed,
 *    implemented non-blocking so main loop remains responsive.
 *  - Driver controls for RPM (fine/coarse adjustments).
 *  - PIDF tuning controls via gamepad2 (cycle through kP/kI/kD/kF and adjust).
 *
 * Controls (new):
 *  - gamepad1.dpad_up/down  : adjust targetRPM by +/-50 (fine)
 *  - gamepad1.dpad_right/left: adjust targetRPM by +/-500 (coarse)
 *  - gamepad1.right_bumper   : toggle automatic multi-shot mode
 *  - gamepad1.a              : set targetRPM to 3300 and begin spinup (existing)
 *  - gamepad1.x              : stop shooter (existing)
 *  - gamepad1.b              : manual single shot trigger (existing)
 *
 * PID tuning (use gamepad2):
 *  - gamepad2.y             : increase selected gain (small step)
 *  - gamepad2.a             : decrease selected gain (small step)
 *  - gamepad2.x             : cycle selected parameter (kP -> kI -> kD -> kF)
 *  - gamepad2.b             : reset PID integral and derivative (useful after changes)
 *
 * NOTES:
 *  - This uses non-blocking timing for auto-shooting so loop remains responsive.
 *  - Tweak feedIntervalMs and feedDurationMs to match your hardware and consistent scoring.
 *  - Test with the robot tethered/disabled first to verify directions & safety.
 */
