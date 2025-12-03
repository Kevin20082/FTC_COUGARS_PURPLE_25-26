package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriveAndShootNoPID", group = "Examples")
public class DriveAndShootNoPID extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorEx Flywheel_Big;
    private DcMotor Flywheel_Small;
    private Servo Top_Servo;

    private VisionPortal visionportal;
    private WebcamName webcam;
    
    boolean inverted = false;

    double Shooting_threshhold = 150;
    double WheelControl = 100;

    double timer1 = 0;
    double timer2 = 0;

    private final ElapsedTime runtime = new ElapsedTime();
    
    private enum ShooterState { IDLE, SPINUP, SHOOT, COOLDOWN}
    private ShooterState shooterState = ShooterState.IDLE;

    private double targetRPM = 0.0;

    private boolean autoMultiShotEnabled = false;   
    private long lastFeedTime = 0L;                
    private long feedIntervalMs = 500L;            
    private long feedDurationMs = 150L;           
    private boolean feeding = false;               
    private long feedStartTime = 0L; 
    
    
    @Override
    public void runOpMode() {
        
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        Flywheel_Big = hardwareMap.get(DcMotorEx.class, " FlyWheel_Big");
        Flywheel_Small = hardwareMap.get(DcMotor.class, "FlyWheel_Small");
        Top_Servo = hardwareMap.get(Servo.class, "Top_Servo");
        
        visionportal = VisionPortal.easyCreateWithDefaults(webcam);
        
        Flywheel_Big.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel_Big.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        Flywheel_Big.setDirection(DcMotorEx.Direction.FORWARD);
        Flywheel_Small.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.update();

        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;
        boolean prevRightBumper = false;
        
        
        while (opModeIsActive()) {
            
            double y1 = -gamepad1.right_stick_y;
            double y2 = -gamepad1.left_stick_y;
            double x1 = -gamepad1.right_stick_x;
            double x2 = -gamepad1.left_stick_x;
            double RWheelPower =((y2-x2)*WheelControl)/100;
            double LWheelPower =((y2+x2)*WheelControl)/100;
            double PowerT = ((-gamepad1.right_trigger));
            double PowerA = ((-gamepad1.left_trigger));
            boolean triggerValue = gamepad1.left_bumper;
            double TPS = (Flywheel_Big.getVelocity(AngleUnit.DEGREES) / 6) * -1;
            
            if (inverted == false) 
            {
                Flywheel_Big.setDirection(DcMotorEx.Direction.FORWARD);
                Flywheel_Small.setDirection(DcMotor.Direction.REVERSE);
            } else {
                Flywheel_Big.setDirection(DcMotorEx.Direction.REVERSE);
                Flywheel_Small.setDirection(DcMotor.Direction.FORWARD);
            }
            
            if (gamepad1.yWasPressed()) {
                if (inverted) {
                    inverted = false;
                } else {
                    inverted = true;
                }
            }
            
            if (TPS < Shooting_threshhold) {PowerA = 0;}
            leftDrive.setPower(LWheelPower);
            rightDrive.setPower(RWheelPower);
            Flywheel_Big.setPower(PowerT);
            Flywheel_Small.setPower(PowerA);

            if (triggerValue == true) {
                Top_Servo.setPosition(.3);
            } else {
                Top_Servo.setPosition(0);
            }



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

            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            
            if (gamepad1.right_bumper && !prevRightBumper) {
                autoMultiShotEnabled = !autoMultiShotEnabled;

                lastFeedTime = System.currentTimeMillis();
                feeding = false;
            }
            prevRightBumper = gamepad1.right_bumper;

            if (gamepad1.a) {
                targetRPM = 3300.0;
                if (shooterState == ShooterState.IDLE) shooterState = ShooterState.SPINUP;

                flywheelBig.onTargetChange(targetRPM);
            } else if (gamepad1.x) {
                targetRPM = 0.0;
                shooterState = ShooterState.IDLE;
                Flywheel_Big.onTargetChange(targetRPM);
            }

            if (gamepad1.b && shooterState == ShooterState.SPINUP) {
                shooterState = ShooterState.SHOOT;
            }
           
           runShooterStateMachineNonBlocking();

            telemetry.addData("State", shooterState);
            telemetry.addData("TargetRPM", "%.0f", targetRPM);
            telemetry.addData("CurrentRPM", "%.0f", getCurrentRPM());
            telemetry.addData("AutoMultiShot", autoMultiShotEnabled);
            telemetry.addData("FeedInterval(ms)", feedIntervalMs);
            telemetry.addData("FeedDuration(ms)", feedDurationMs);
            telemetry.addData("Left Power", LWheelPower);
            telemetry.addData("Right Power", RWheelPower);
            telemetry.addData("FlyWheel Big", PowerT);
            telemetry.addData("FlyWheel Small", PowerA);
            telemetry.addData("Top Servo", triggerValue);
            telemetry.addData("Inverted: ", Inverted);
            telemetry.update();
        }
    }

     private void runShooterStateMachineNonBlocking() {

        double currentRPM = getCurrentRPM();

        
        if (shooterState == ShooterState.SPINUP) {
            double power = (targetRPM > 0.0) ? 0.8 : 0.0;  
            Flywheel_Big.setPower(power);

            if (!feeding) Flywheel_Small.setPower(0.0);
            
            if (Math.abs(currentRPM - targetRPM) < 150.0 && targetRPM > 0.0) {
            
                if (autoMultiShotEnabled) {
                    shooterState = ShooterState.SPINUP;
                    if (lastFeedTime == 0L) lastFeedTime = System.currentTimeMillis();
                } else {

                    shooterState = ShooterState.SHOOT;
                }
            }
        } else if (shooterState == ShooterState.IDLE) {
            flywheelBig.setPower(0.0);
            flywheelSmall.setPower(0.0);
        } else if (shooterState == ShooterState.SHOOT) {
            if (!feeding) {
                feeding = true;
                feedStartTime = System.currentTimeMillis();
                flywheelSmall.setPower(1.0);
            }
        
            if (feeding) {
                if (System.currentTimeMillis() - feedStartTime >= feedDurationMs) {
                    
                    flywheelSmall.setPower(0.0);
                    feeding = false;
                
                    shooterState = ShooterState.COOLDOWN;
                    lastFeedTime = System.currentTimeMillis();
                }
            }
        } else if (shooterState == ShooterState.COOLDOWN) {
           
            if (!feeding) {
                feeding = true;
                feedStartTime = System.currentTimeMillis();
            }
            
            if (System.currentTimeMillis() - feedStartTime >= feedDurationMs) {
                feeding = false;
               
                shooterState = (targetRPM > 0.0) ? ShooterState.SPINUP : ShooterState.IDLE;
            }
        }

    
        if (autoMultiShotEnabled && targetRPM > 0.0 && Math.abs(currentRPM - targetRPM) < 150.0) {
            long now = System.currentTimeMillis();

            
            if (!feeding && (now - lastFeedTime >= feedIntervalMs)) {
                feeding = true;
                feedStartTime = now;
                flywheelSmall.setPower(1.0);
            }

            
            if (feeding && (now - feedStartTime >= feedDurationMs)) {
                feeding = false;
                flywheelSmall.setPower(0.0);
                lastFeedTime = now;
                
                shooterState = ShooterState.SPINUP;
            }
        }
    }

    
    private double getCurrentRPM() {
        double currentRPM = 0.0;
        try {
            currentRPM = Math.abs(flywheelBig.getVelocity());
        } catch (Exception ignored) {
            currentRPM = 0.0;
        }
        return currentRPM;
    }
}      
