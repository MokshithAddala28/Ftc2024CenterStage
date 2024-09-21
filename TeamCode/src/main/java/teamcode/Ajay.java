package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous
public class Ajay extends LinearOpMode {
    DistanceSensor frontSensor;
    OpenCvWebcam webcam1 = null;
    public static DcMotorEx leftFront;
    public static DcMotorEx rightFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightRear;

    public static DcMotorEx leftRobotArm;
    public static DcMotorEx rightRobotArm;
    public static Servo rightClaw;
    public static Servo leftClaw;
    public static Servo wrist;
    private static int value = 0;
    public static double Dist;
    int targetPosition1 = 200;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRobotArm = hardwareMap.get(DcMotorEx.class, "rightRobotArm");
        leftRobotArm = hardwareMap.get(DcMotorEx.class, "leftRobotArm");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        frontSensor = hardwareMap.get(DistanceSensor.class, "frontSensor");
        leftRobotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRobotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRobotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRobotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Drivetrain.init(leftFront, rightFront, leftRear, rightRear);
        Arm.init(rightRobotArm, leftRobotArm, rightClaw, leftClaw, wrist);

        Arm.closeClaw();
        waitForStart();
    Drivetrain.encoderForward(20);
    sleep(500);
    Drivetrain.encoderTurn(90);
    sleep(400);
    Drivetrain.encoderForward(20);
    sleep(500);
    Drivetrain.encoderTurn(-90);
    sleep(400);
    Drivetrain.encoderForward(20);
    //Forward 10 inches, Right turn, Forward 5 inches, Left turn, Forward 7 inches
    }
}
