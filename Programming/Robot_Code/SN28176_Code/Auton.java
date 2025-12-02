package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auton", group = "Autonomous Opmode")
public class Auton extends LinearOpMode {

    // Autonomous tuning constants
    final double DESIRED_DISTANCE = 60.0; // inches (example)
    final double SPEED_GAIN = 0.05; // Speed adjustment gain
    final double TURN_GAIN = 0.01; // Turn adjustment gain
    final double MAX_AUTO_SPEED = 0.75; // Maximum speed during autonomous
    final double MIN_AUTO_SPEED = 0.5; // Minimum speed during autonomous

    // Drive and shooter hardware
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx flywheelBig;
    private DcMotorEx flywheelSmall;

    // Vision
    private static final boolean USE_WEBCAM = true; // Set false to use built-in camera
    private int DESIRED_TAG_ID = -1; // ID of the AprilTag to detect
    private int DESIRED_ORDER_TAG_ID = -1; // ID of the order tag to detect
    private static double dist = 9999;
    private static double angle = 0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection desiredTag = null;
    private AprilTagDetection orderTag = null;
    private boolean targetFound = false;
    private boolean orderTagFound = false;

    // Autonomous state machine
    private enum AutoState { SEARCH, APPROACH_GOAL, SHOOT, GO_TO_LOADING, LOADING, RETURN_TO_GOAL }
    private AutoState autoState = AutoState.SEARCH;

    // Shooter state machine and PID
    private enum ShooterState { IDLE, SPINUP, SHOOT, COOLDOWN }
    private ShooterState shooterState = ShooterState.IDLE;
    private double targetRPM = 0.0;
    private PIDF flywheelPid = new PIDF(0.001, 0.0, 0.0005, 0.0);

    // Misc
    private ElapsedTime runtime = new ElapsedTime();
    private long stateStartTime = 0; // milliseconds
    private boolean teamIsRed = true; // Set based on alliance

    @Override
    public void runOpMode() {
        // Initialize vision and hardware
        initAprilTag();

        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        flywheelBig = hardwareMap.get(DcMotorEx.class, "Flywheel_Big");
        flywheelSmall = hardwareMap.get(DcMotorEx.class, "Flywheel_Small");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        flywheelBig.setDirection(DcMotor.Direction.REVERSE);
        flywheelSmall.setDirection(DcMotor.Direction.FORWARD);

        // Configure which tags we are looking for based on team
        InitiateTeamTags(teamIsRed ? "RED" : "BLUE");

        // Select camera
        if (USE_WEBCAM) {
            visionPortal.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            visionPortal.setActiveCamera(BuiltinCameraDirection.BACK);
        }

        telemetry.addData("Status", "Init complete. Team: %s", teamIsRed ? "RED" : "BLUE");
        telemetry.addData("Desired Tag ID", DESIRED_TAG_ID);
        telemetry.addData("Order Tag ID", DESIRED_ORDER_TAG_ID);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // small forward nudge
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(1000);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);

        targetFound = false;
        orderTagFound = false;

        while (opModeIsActive()) {
            // Retrieve latest detections (api dependent on processor implementation)
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            if (detections != null) {
                Optional<AprilTagDetection> optDesired = detections.stream()
                        .filter(tag -> tag.id == DESIRED_TAG_ID)
                        .findFirst();
                Optional<AprilTagDetection> optOrder = detections.stream()
                        .filter(tag -> tag.id == DESIRED_ORDER_TAG_ID)
                        .findFirst();

                desiredTag = optDesired.orElse(null);
                orderTag = optOrder.orElse(null);
            } else {
                desiredTag = null;
                orderTag = null;
            }

            // Basic auto state machine
            switch (autoState) {
                case SEARCH:
                    if (desiredTag != null) {
                        dist = desiredTag.getDistance();
                        // compute approximate horizontal angle (example)
                        angle = Math.toDegrees(Math.atan2(desiredTag.pose.x, desiredTag.pose.z));
                        targetFound = true;
                        autoState = AutoState.APPROACH_GOAL;
                    } else {
                        // rotate in place slowly searching for tag
                        leftDrive.setPower(0.3);
                        rightDrive.setPower(-0.3);
                    }
                    break;

                case APPROACH_GOAL:
                    if (targetFound && desiredTag != null) {
                        dist = desiredTag.getDistance();
                        angle = Math.toDegrees(Math.atan2(desiredTag.pose.x, desiredTag.pose.z));

                        double speedCommand = SPEED_GAIN * (dist - DESIRED_DISTANCE);
                        speedCommand = Range.clip(speedCommand, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                        double turnCommand = TURN_GAIN * angle;
                        turnCommand = Range.clip(turnCommand, -0.3, 0.3);

                        double leftPower = speedCommand + turnCommand;
                        double rightPower = speedCommand - turnCommand;

                        leftDrive.setPower(leftPower);
                        rightDrive.setPower(rightPower);

                        if (dist <= DESIRED_DISTANCE + 5.0) {
                            leftDrive.setPower(0.0);
                            rightDrive.setPower(0.0);
                            autoState = AutoState.SHOOT;
                            stateStartTime = (long) runtime.milliseconds();
                        }
                    } else {
                        autoState = AutoState.SEARCH;
                    }
                    break;

                case SHOOT:
                    // Ensure shooter FSM runs while in SHOOT state
                    runShooterStateMachine();
                    // After a fixed time or condition, go to loading
                    if (runtime.milliseconds() - stateStartTime > 5000) { // Shoot for 5 seconds
                        autoState = AutoState.GO_TO_LOADING;
                    }
                    break;

                case GO_TO_LOADING:
                    // Implement path to loading zone
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    autoState = AutoState.LOADING;
                    break;

                case LOADING:
                    // Implement loading behavior (example placeholder)
                    // After loading, return to goal
                    autoState = AutoState.RETURN_TO_GOAL;
                    break;

                case RETURN_TO_GOAL:
                    // Implement return path to goal (placeholder)
                    leftDrive.setPower(0.0);
                    rightDrive.setPower(0.0);
                    // End autonomous
                    requestOpModeStop();
                    break;
            }

            telemetry.addData("Auto State", autoState);
            telemetry.addData("Target Found", targetFound);
            telemetry.addData("Desired Tag", DESIRED_TAG_ID);
            telemetry.addData("Dist", dist);
            telemetry.addData("Angle", angle);
            telemetry.update();
        }
    }

    private void initAprilTag() {
        visionPortal = VisionPortal.getInstance();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagSize(0.166) // Tag size in meters
                .setFx(578.272)   // Focal length x
                .setFy(578.272)   // Focal length y
                .setCx(402.145)   // Principal point x
                .setCy(221.506)   // Principal point y
                .build();
        visionPortal.addProcessor(aprilTagProcessor);
    }

    private void InitiateTeamTags(String teamColor) {
        if (teamColor.equals("RED")) {
            DESIRED_TAG_ID = 1; // Example ID for RED
            DESIRED_ORDER_TAG_ID = 4; // Example order tag ID for RED
        } else {
            DESIRED_TAG_ID = 2; // Example ID for BLUE
            DESIRED_ORDER_TAG_ID = 5; // Example order tag ID for BLUE
        }
    }

    private void setManualExposrure(int gainValue, int exposureTimeMs) {
        GainControl gainControl;
        ExposureControl exposureControl;

        if (USE_WEBCAM) {
            gainControl = visionPortal.getActiveCamera().getGainControl();
            exposureControl = visionPortal.getActiveCamera().getExposureControl();
        } else {
            gainControl = visionPortal.getActiveCamera().getGainControl();
            exposureControl = visionPortal.getActiveCamera().getExposureControl();
        }

        if (gainControl != null && exposureControl != null) {
            gainControl.setGain(gainValue);
            exposureControl.setExposureTime(exposureTimeMs, TimeUnit.MILLISECONDS);
        }
    }

    /**
     * Shooter finite state machine.
     * Controls spin-up, firing, and cool-down behavior.
     */
    private void runShooterStateMachine() {
        double currentRPM = 0.0;
        try {
            // Get current flywheel RPM
            currentRPM = Math.abs(flywheelBig.getVelocity());
        } catch (Exception ignored) {
            // If velocity reading fails, assume 0 RPM
            currentRPM = 0.0;
        }

        switch (shooterState) {
            case IDLE:
                // Stop flywheel and intake
                flywheelBig.setPower(0.0);
                flywheelSmall.setPower(0.0);
                break;

            case SPINUP:
                // Use PID controller to reach target RPM
                double powerCommand = flywheelPid.update(targetRPM, currentRPM);
                powerCommand = Range.clip(powerCommand, 0.0, 1.0);
                flywheelBig.setPower(powerCommand);

                // Keep intake off while spinning up
                flywheelSmall.setPower(0.0);

                if (Math.abs(currentRPM - targetRPM) < 150.0 && targetRPM > 0) {
                    shooterState = ShooterState.SHOOT;
                }
                break;

            case SHOOT:
                // Run intake briefly to feed projectile into shooter
                double intakePower = 1.0; // tune as needed
                flywheelSmall.setPower(intakePower);
                sleep(150);
                flywheelSmall.setPower(0.0);
                shooterState = ShooterState.SPINUP; // Return to SPINUP after shooting
                break;

            case COOLDOWN:
                // Placeholder for cooldown behavior
                flywheelBig.setPower(0.0);
                flywheelSmall.setPower(0.0);
                shooterState = ShooterState.IDLE;
                break;
        }
    }

    // Simple PIDF implementation used by the shooter
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

            double error = target - actual;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            return (kP * error) + (kI * integral) + (kD * derivative) + (kF * target);
        }

        public void reset() {
            integral = 0.0;
            lastError = 0.0;
            lastTime = System.nanoTime() / 1e9;
        }
    }
}