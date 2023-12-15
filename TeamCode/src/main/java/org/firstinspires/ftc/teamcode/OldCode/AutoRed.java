package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) drive:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) strafe:  Strafing right and left                     Left-joystick Right and Left
 * 3) turn:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="AutoRed", group="Robot")

public class AutoRed extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedPropp.tflite";
    private static final String[] LABELS = {
       "Pixel",
       "RedProp"
    };
    
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime      = new ElapsedTime();
    private DcMotor leftFrontDrive   = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightBackDrive   = null;

    private Servo lever              = null;
    
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    
    private static final double UP   = 0.3;
    private static final double DOWN = 0;
    private static final double MA   = 0.2;
    
    
    
    double drive;
    double strafe;
    double turn;
    int randomization = 0;
    double bat = 0.97;
    

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor0");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "motor1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor2");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "motor3");
        
        lever           = hardwareMap.get(Servo.class, "servo0");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        // Wait for the game to start (driver presses PLAY)
        initTfod();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while(opModeInInit()){
            telemetryTfod();
        }

        
        waitForStart();
        runtime.reset();
        
        boolean triggerDown = false;
        double doorPosition = DOWN;
        
       
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetryTfod();
            
            // randomization = 
            if(randomization == 0){
                if(runtime.milliseconds() < 600){
                    drive = 0.5 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 800 && runtime.milliseconds() < 1600){
                    drive = 0.45 * bat;
                    strafe = 0 * bat;
                    turn = -0.3 * bat;
                }
                
                else if(runtime.milliseconds() < 2600 && runtime.milliseconds() > 1700){
                    drive = -0.45 * bat;
                    strafe = 0 * bat;
                    turn = 0.3 * bat;
                }
                
                else if(runtime.milliseconds() < 3400 && runtime.milliseconds() > 2800){
                    drive = -0.6 * bat;
                    strafe = 0.01 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 3600 && runtime.milliseconds() < 5900){
                    drive = 0.5 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() < 13300 && runtime.milliseconds() > 6300){
                    drive = 0 * bat;
                    strafe = -0.5 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 13300 && runtime.milliseconds() < 14500){
                    drive = -0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                //end 25 point
                else if(runtime.milliseconds() < 14800 && runtime.milliseconds() > 14700){
                    drive = -0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
               /* 
                else if(runtime.milliseconds() > 15000 && runtime.milliseconds() < 15800){
                    drive = 0 * bat;
                    strafe = -0.5 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() < 15800 && runtime.milliseconds() > 14500){
                    drive = 0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 16000 && runtime.milliseconds() < 16500){
                    drive = 0 * bat;
                    strafe = 0.5 * bat;
                    turn = 0 * bat;
                }//new
                
                else if(runtime.milliseconds() > 16000 && runtime.milliseconds() < 17200){
                    drive = 0 * bat;
                    strafe = 0 * bat;
                    turn = 0.5 * bat;
                }
                */
                else{
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                }
            }
            
            else if(randomization == 1){
            
                if(runtime.milliseconds() < 1300){
                    drive = 0.5 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() < 2300 && runtime.milliseconds() > 1500){
                    drive = 0 * bat;
                    strafe = 0.5 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 2500 && runtime.milliseconds() < 3400){
                    drive = 0.5 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() < 11000 && runtime.milliseconds() > 4100){
                    drive = 0 * bat;
                    strafe = -0.5 * bat;
                    turn = -0.005 * bat;
                }
                
                //end 25 point
                else if(runtime.milliseconds() < 12200 && runtime.milliseconds() > 11200){
                    drive = -0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                //yellow
                
                
                else if(runtime.milliseconds() > 12600 && runtime.milliseconds() < 12400){
                    drive = 0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() < 13600 && runtime.milliseconds() > 12600){
                    drive = 0 * bat;
                    strafe = 0.5 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 13600 && runtime.milliseconds() < 14400){
                    drive = 0 * bat;
                    strafe = 0 * bat;
                    turn = 0.5 * bat;
                }
                
                else if(runtime.milliseconds() < 15900 && runtime.milliseconds() > 14900){
                    drive = 0 * bat;
                    strafe = -0.5 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 16000 && runtime.milliseconds() < 16300){
                    drive = 0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                else if(runtime.milliseconds() > 16000 && runtime.milliseconds() > 16300){
                    lever.setPosition(1);
                }
                
                else{
                    
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                    
                }
            }
            else if (randomization == 2){
                if(runtime.milliseconds() < 600){
                    drive = 0.5 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 800 && runtime.milliseconds() < 1500){
                    drive = 0.45 * bat;
                    strafe = 0 * bat;
                    turn = 0.3 * bat;
                }
                
                else if(runtime.milliseconds() < 2600 && runtime.milliseconds() > 1700){
                    drive = -0.45 * bat;
                    strafe = 0 * bat;
                    turn = -0.3 * bat;
                }
                
                else if(runtime.milliseconds() < 3400 && runtime.milliseconds() > 2900){
                    drive = -0.6 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 3600 && runtime.milliseconds() < 5800){
                    drive = 0.5 * bat;
                    strafe = 0.01 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() < 12500 && runtime.milliseconds() > 6300){
                    drive = 0 * bat;
                    strafe = -0.5 * bat;
                    turn = -0.005 * bat;
                }
                
                //end 25 point
                else if(runtime.milliseconds() < 13400 && runtime.milliseconds() > 12700){
                    drive = -0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                /*else if(runtime.milliseconds() > 13600 && runtime.milliseconds() < 13700){
                    drive = 0.25 * bat;
                    strafe = 0 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 13900 && runtime.milliseconds() < 14500){
                    drive = 0 * bat;
                    strafe = -0.5 * bat;
                    turn = 0 * bat;
                }
                
                else if(runtime.milliseconds() > 16000 && runtime.milliseconds() < 16900){
                    drive = 0 * bat;
                    strafe = 0 * bat;
                    turn = - 0.5 * bat;
                }
                */
                else{
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                }
            }
            if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
            }
            
            // crawlTo(0, 48, 0, runtime.milliseconds() / 1000.0);
            
            double leftFrontPower  = - drive + strafe - turn;
            double rightFrontPower =   drive + strafe - turn;
            double leftBackPower   =   drive + strafe + turn;
            double rightBackPower  = - drive + strafe + turn;

            //Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            
            // leftFrontDrive.setPower(1);
            // rightFrontDrive.setPower(-1);
            // leftBackDrive.setPower(-1);
            // rightBackDrive.setPower(1);
            
            //doorRight.setPosition(doorPosition);
            

            // Show the elapsed game time and wheel power.
        }
        
        visionPortal.close();
    }
    
    /*public double profile(double distance, double maxAccel, double maxSpeed, double t){
        double totalTime = (maxSpeed/maxAccel) + (distance - (maxSpeed * maxSpeed) / (2 * maxAccel) );
        double p1 = maxSpeed / (2 * maxAccel) * (abs(distance + 1) / 2 - abs(distance - 1) / 2);
        double p2 = totalTime - maxSpeed / (2 * maxAccel);
        
        return - maxSpeed * (abs(t-p1) + abs(t-p2) - p2 - p1 );
    }
    
    public boolean crawlTo(double x, double y, double headingDelta, double t){
        
        x = x / 86;
        y = y / 86;
        
        drive = profile(y, MA, 0.5, t);
        strafe = profile(x, MA, 0.5, t);
        
        if((drive / abs(drive) == -1 && y >=0)||(drive / abs(drive) == 1 && y <= 0)){
            drive = 0;
        }
        if((strafe / abs(strafe) == -1 && y >=0)||(strafe / abs(strafe) == 1 && y <= 0)){
            strafe = 0;
        }
        
        double totalTime = (1/MA) + (y - 1.0 / (2 * MA) );
        
        turn = headingDelta / totalTime;
        
        return t == totalTime;
    }
    public double abs(double x){
        if(x < 0){
            return - x;
        }
        return x;
    }*/
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().setModelFileName(TFOD_MODEL_FILE).build();
        
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.5f);

    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            
            randomization = posFind(x);
            
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("",randomization);
            telemetry.update();
        
            
        }
        if(currentRecognitions.size() == 0){
            randomization = 0;
        }// end for() loop
        
    } 
    private int posFind(double x){
        // int pos = 0;
        
         if(x >= 490){
            return 2;
        }
        else if (x >= 1){
            return 1;
        }
        
        return 0;
    }
}
