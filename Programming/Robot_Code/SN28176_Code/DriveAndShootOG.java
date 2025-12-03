package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveAndShoot", group = "Examples")
public class DriveAndShoot extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorEx Flywheel_Big;
    private DcMotor FlyWheel_Small;
    private Servo Top_Servo;

    private VisionPortal visionportal;
    private WebcamName webcam;
    
    //boolean Harry = false;
    boolean Inverted = false;

    double Shooting_threshhold = 150;
    double WheelControl = 100;

    double timer1 = 0;
    double timer2 = 0;
    
    @Override
    public void runOpMode() {
        
        //webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        Flywheel_Big = hardwareMap.get(DcMotorEx.class, " FlyWheel_Big");
        Flywheel_Small = hardwareMap.get(DcMotor.class, "FlyWheel_Small");
        Top_Servo = hardwareMap.get(Servo.class, "Top_Servo");
        
        //visionportal = VisionPortal.easyCreateWithDefaults(webcam);
        
        Flywheel_Big.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel_Big.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        Flywheel_Big.setDirection(DcMotorEx.Direction.FORWARD);
        Flywheel_Small.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        
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
            double TPS = (FlyWheel_Big.getVelocity(AngleUnit.DEGREES) / 6) * -1;
            
            if (Inverted == false) 
            {
                Flywheel_Big.setDirection(DcMotorEx.Direction.FORWARD);
                Flywheel_Small.setDirection(DcMotor.Direction.REVERSE);
            } else {
                Flywheel_Big.setDirection(DcMotorEx.Direction.REVERSE);
                Flywheel_Small.setDirection(DcMotor.Direction.FORWARD);
            }
            
            //if (Harry)
           // {
            //    RWheelPower =((y1-x2)*WheelControl)/100;
            //    LWheelPower =((y1+x2)*WheelControl)/100;
           //}
            
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
            
            Wheel_Motors_Speed_Control();
            Fly_Wheel_Motors_Speed_Control();
            
            telemetry.addData("Left Power", LWheelPower);
            telemetry.addData("Right Power", RWheelPower);
            telemetry.addData("FlyWheel Big", PowerT);
            telemetry.addData("FlyWheel Small", PowerA);
            telemetry.addData("Top Servo", triggerValue);
            telemetry.addData("Fly Wheel Shooting Speed", Shooting_threshhold+" RPM");
            telemetry.addData("Fly Wheel RPM", TPS);
           // telemetry.addData("HarryMode: ", Harry);
            telemetry.addData("Inverted: ", Inverted);
            telemetry.update();
        }
    }
    
    public void Fly_Wheel_Motors_Speed_Control () {
        if (gamepad1.dpad_up) {
            if  (timer1 == 0) {
                if (Shooting_threshhold == 150) {
                   Shooting_threshhold = 190;
                   timer1 = 1500;
                }
            } else {
                timer1-=5;
            }
        }
        if (gamepad1.dpad_down) {
            if  (timer1 == 0) {
                if (Shooting_threshhold > 150) {
                   Shooting_threshhold = 150;
                   timer1 = 1500;
                }
            } else {
                timer1-=5;
            }
        }
    }
    
    public void Wheel_Motors_Speed_Control () {
        //if (gamepad1.dpad_right) {
           // if  (timer2 == 0) {
            //    if (Harry) {Harry = false;} else {Harry = true;}
            //    timer2 = 3000;
          //  } else {
            //    timer2-=1;
           // }
      //  }
        if (gamepad1.xWasPressed()) {
            if (Inverted) {Inverted = false;} else {Inverted = true;}
        }
    }
}

