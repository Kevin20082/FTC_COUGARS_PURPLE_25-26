package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual_Drive", group = "Examples")
public class Manual_Drive extends LinearOpMode {

    private DcMotorEx LeftWheels;
    private DcMotorEx RightWheels;
    private DcMotorEx Shooter;
    private DcMotorEx Turret;
    private DcMotor Intake;
    private Servo Blocker;
    private Servo Hood;

    private VisionPortal visionportal;
    private WebcamName Webcam;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int DESIRED_TAG_ID = -1;
    
    private boolean USE_WEBCAM = true;
    private boolean teamIsRed = true;
    private boolean Inverted = false;
    private boolean Manual_Aiming = false;
    private boolean Servo_Blocking = false;
    
    private int Max_Velocity = 5500;
    private double Velocity_Gain = 1.0 / Max_Velocity;
    
    // Driving Yap
    private double kP_Drive = .001;
    private double kI_Drive = 0;
    private double kD_Drive = 0;
    private double integral_Right_Wheels = 0, integral_Left_Wheels = 0;
    private double lastError_Right_Wheels = 0, lastError_Left_Wheels = 0;
    
    // Aiming Yap
    private static double Goal_Distance = 0;
    private static double Goal_Angle = 0;
    private double Goal_Search_Speed = 0.25;
    private double Goal_Aim_Speed = 0.5;
    private double Manual_Aim_Sensitivity = .1;
    
    private double kP_Aiming = .001;
    private double kI_Aiming = 0;
    private double kD_Aiming = 0;
    private double integral_Aiming = 0;
    private double lastError_Aiming = 0;
    
    // Blocking Yap
    private boolean Servo_Blocking = false;
    private double Servo_Closed_Angle = .3;
    private double Servo_Opened_Angle = .8;
    
    // Intake Yap
    private boolean Intake_Intaking = false;
    private double Intake_On_Strength = 1;
    
    // Shooting Yap
    private int TargetShootingRPM = 3000;
    private int Current_Shooter_RPM = 0;
    
    private double kP_Shooter = 0.0008;
    private double kI_Shooter = 0.0000005;
    private double kD_Shooter = 0.0001;
    private double integral_Shooter = 0;
    private double lastError_Shooter = 0;
    
    // Telemetry Yap
    private double Right_Wheels_Power_Telemetry = 0;
    private double Left_Wheels_Power_Telemetry = 0;
    private boolean Servo_Blocking_Telemetry = false;
    private boolean Intake_Intaking_Telemetry = false;
    
    @Override
    public void runOpMode() {
        
        LeftWheels = hardwareMap.get(DcMotorEx.class, "Left Motor");
        RightWheels = hardwareMap.get(DcMotorEx.class, "Right Motor");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Hood = hardwareMap.get(Servo.class, "Hood");
        
        LeftWheels.setDirection(DcMotorEx.Direction.REVERSE);
        RightWheels.setDirection(DcMotorEx.Direction.FORWARD);
        Shooter.setDirection(DcMotorEx.Direction.FORWARD);
        Turret.setDirection(DcMotorEx.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        
        LeftWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        initAprilTag();
        if (USE_WEBCAM) setManualExposure(6, 150);
        
        InitiateTeamTags(teamIsRed ? "RED" : "BLUE");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            double y1 = gamepad1.right_stick_y;
            double y2 = gamepad1.left_stick_y;
            double x1 = gamepad1.right_stick_x;
            double x2 = gamepad1.left_stick_x;
            boolean RBPress = gamepad1.right_bumper;
            boolean LBPress = gamepad1.left_bumper;
            double RTPress_Value = gamepad1.right_trigger;
            
            double Base_Right_Power = (y2-x2);
            double Base_Left_Power = (y2+x2);
            double Turret_Turn_Strength = y1;
            
            if (gamepad1.xWasPressed()) {Inverted = !Inverted; UpdateInverted();}
            if (gamepad1.yWasPressed()) {Manual_Aiming = !Manual_Aiming;}
            
            Drive(Base_Right_Power, Base_Left_Power);
            Aim(Turret_Turn_Strength);
            Servo_Control(RBPress);
            Intake_Control(LBPress);
            Shooter_Control(RTPress_Value);
            Set_Hood_Angle();
            
            UpdateTelemetry();
        }
    }
    
    public void Set_Hood_Angle()
    {
        
    }
    
    public void Shooter_Control(double Press)
    {
        Current_Shooter_RPM = Shooter.getVelocity();
        
        if (Press < 0.1) 
        {
            Shooter.setPower(0);
            integral_Shooter = 0;
            lastError_Shooter = 0;
            return;
        }
        
        double Shooter_Power = Shooter_PID();
    
        Shooter.setPower(Shooter_Power);
    }
    
    public double Shooter_PID()
    {
        double error = TargetShootingRPM - currentRPM;
        integral_Shooter += error;
        integral_Shooter = Range.clip(integral_Shooter, -TargetShootingRPM, TargetShootingRPM);
        double derivative = error - lastError_Shooter;
        lastError_Shooter = error;
    
        double P = kP_Shooter * error;
        double I = kI_Shooter * integral_Shooter;
        double D = kD_Shooter * derivative;
    
        double FF = Velocity_Gain * TargetShootingRPM;
    
        double Power = P + I + D + FF;
        Power = Range.clip(Power, 0, 1);
        
        return Power;
    }
    
    public void Intake_Control(boolean Pressed)
    {
        if (Pressed)
        {
            Intake.setPower(Intake_On_Strength);
            Intake_Intaking_Telemetry = true;
        } else {
            Intake.setPower(0);
            Intake_Intaking_Telemetry = false;
        }
    }
    
    public void Servo_Control(boolean Pressed)
    {
        if (Pressed && !RBLastPressed) {
            Servo_Blocking; = !Servo_Blocking;
            Servo_Blocking_Telemetry = Servo_Blocking;
            Update_Servo();
        }
    
        RBLastPressed = Pressed;
    }
    
    public void Update_Servo()
    {
        if (Servo_Blocking) 
        {Blocker.setPosition(Servo_Closed_Angle);} 
        else 
        {Blocker.setPosition(Servo_Opened_Angle);}
    }
    
    public void Aim(Turn_Strength)
    {
        if (Manual_Aiming == false) 
        {
            Camera_Look();
            UpdateCameraEvents();
        } else {
            Manual_Aiming_Control(Turn_Strength);
        }
    }
    
    public void Manual_Aiming_Control(Turn_Strength) 
    {
        Turret.setPower(Manual_Aim_Sensitivity * Turn_Strength)
    }
    
    public void Camera_Look()
    {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections != null) {
            Optional<AprilTagDetection> optDesired = detections.stream()
                    .filter(tag -> tag.id == DESIRED_TAG_ID)
                    .findFirst();
            desiredTag = optDesired.orElse(null);
        } else {
            desiredTag = null;
        }
    }
    
    public void UpdateCameraEvents()
    {
        if (desiredTag != null)
        {
            Goal_Distance = desiredTag.getDistance();
            Goal_Angle = Math.toDegrees(Math.atan2(desiredTag.pose.x, desiredTag.pose.z));
            Aim_Shooter_Camera();
            
        } else {Look__For_Goal();}
    }
    
    public void Aim_Shooter_Camera()
    {
        if (desiredTag != null)
        {
            integral_Aiming += Goal_Angle;
            double derivative = Goal_Angle - lastError_Aiming;
            lastError_Aiming = Goal_Angle;
    
            double Turret_Turn_Power = (kP_Aiming * Goal_Angle) + (kI_Aiming * integral_Aiming) + (kD_Aiming * derivative);
    
            Turret_Turn_Power = Range.clip(Turret_Turn_Power, -Goal_Aim_Speed, Goal_Aim_Speed);
    
            Turret.setPower(Turret_Turn_Power);
        }
    }
    
    public void Look__For_Goal()
    {
        Turret.setPower(Goal_Search_Speed);
    }
    
    private void InitiateTeamTags(String teamColor)
    {
        if (teamColor != null && teamColor.equalsIgnoreCase("RED")) { DESIRED_TAG_ID = 24; } else { DESIRED_TAG_ID = 20; }
    }
    
    public void UpdateInverted() 
    {
        if (Inverted == false) 
        {
            Intake.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            Intake.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }
    
    public void Drive(double Base_Right_Power, double Base_Left_Power)
    {
        double[] Drive_Values = Get_Drive_PID_Values(Base_Right_Power, Base_Left_Power);
        
        double Right_Power = Drive_Values[0];
        double Left_Power = Drive_Values[1];
        
        Right_Wheels_Power_Telemetry = Right_Power;
        Left_Wheels_Power_Telemetry = Left_Power;
        
        RightWheels.setPower(Right_Power);
        LeftWheels.setPower(Left_Power);
    }
    
    public double[] Get_Drive_PID_Values(double Base_Right_Power, double Base_Left_Power)
    {
        double Right_Velocity = RightWheels.getVelocity();
        double Left_Velocity = LeftWheels.getVelocity();

        double targetRightVel = Base_Right_Power * Max_Velocity;
        double targetLeftVel = Base_Left_Power * Max_Velocity;
        
        double rightFF = Velocity_Gain * targetRightVel;
        double leftFF  = Velocity_Gain * targetLeftVel;
        
        double rightCorrection = Calculate_Corrections(true, targetRightVel, Right_Velocity);
        double leftCorrection = Calculate_Corrections(false, targetLeftVel, Left_Velocity);
        
        double finalRightPower = rightFF + rightCorrection;
        double finalLeftPower  = leftFF  + leftCorrection;
    
        finalRightPower = Math.max(-1.0, Math.min(1.0, finalRightPower));
        finalLeftPower = Math.max(-1.0, Math.min(1.0, finalLeftPower));
    
        double[] FinalValues = new double[2];
        FinalValues[0] = finalRightPower;
        FinalValues[1] = finalLeftPower;
        return FinalValues;
    }
    
    public double Calculate_Corrections(boolean Is_Right_Wheels, double target, double current) 
    {
        if (Is_Right_Wheels)
        {
            double error = target - current;
            integral_Right_Wheels += error;
            double derivative = error - lastError_Right_Wheels;
            lastError_Right_Wheels = error;
    
            return (kP_Drive * error) + (kI_Drive * integral_Right_Wheels) + (kD_Drive * derivative);
        } else {
            double error = target - current;
            integral_Left_Wheels += error;
            double derivative = error - lastError_Left_Wheels;
            lastError_Left_Wheels = error;
    
            return (kP_Drive * error) + (kI_Drive * integral_Left_Wheels) + (kD_Drive * derivative);
        }
    }
    
    private void initAprilTag() 
    {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    
    private void setManualExposure(int exposureMS, int gain) 
    {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    
    public void UpdateTelemetry()
    {
        telemetry.addData("Right Wheels Power: ", Right_Wheels_Power_Telemetry);
        telemetry.addData("Left Wheels Power: ", Left_Wheels_Power_Telemetry);
        telemetry.addData("Goal Distance: ", Goal_Distance);
        telemetry.addData("Goal Angle: ", Goal_Angle);
        telemetry.addData("Manual Aiming: ", Manual_Aiming);
        telemetry.addData("Servo: ", Servo_Blocking_Telemetry);
        telemetry.addData("Intake: ", Intake_Intaking_Telemetry);
        telemetry.addData("Shooter RPM: ", Current_Shooter_RPM);
        telemetry.addData("TargetShootingRPM: ", TargetShootingRPM);
        
        telemetry.update();
    }
}
