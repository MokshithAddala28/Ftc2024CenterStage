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
public class OpenCVRedLeft extends LinearOpMode {
    OpenCvWebcam webcam1 = null;
    DistanceSensor frontSensor;

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



        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new ExamplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                //startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT);
                //startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        if (value == 1) {

            Drivetrain.encoderForward(20);
            Drivetrain.encoderHalfTurnLeft(305);
            sleep(1000);
            //Drivetrain.encoderForward(5);
            Drivetrain.encoderForward(-8);
            Drivetrain.encoderTurn(-155);
            sleep(1000);
            // Drivetrain.encoderStrafe(7);
            Drivetrain.encoderForward(46);
            sleep(1000);
            Drivetrain.encoderTurn(-285);
            sleep(500);
            Drivetrain.encoderForward(125);
         //   sleep(1000);
            Drivetrain.encoderStrafe(-32);
       //     sleep(1000);
            rightRobotArm.setTargetPosition(targetPosition1);
            rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRobotArm.setTargetPosition(-targetPosition1);
            leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRobotArm.setPower(0.5);
            leftRobotArm.setPower(-0.5);

            while (distance() > 4)  {  //the robot has to be closer to the backdrop
                Drivetrain.encoderForward(1);
            }
            Arm.wristMid();
            sleep(1000);
            Arm.openClaw();
            sleep(1000);
            Arm.wristUp();
            sleep(1000);
            Drivetrain.encoderForward(-3);
            Drivetrain.encoderStrafe(33);
         //   Drivetrain.encoderForward(5);
          //  sleep(1000);


           /* Old Code
            Drivetrain.encoderForward(23);

            sleep(1000);
            Drivetrain.encoderTurn(-130);
            sleep(1000);
            Arm.wristLow();
            sleep(1000);
            Arm.openClaw();
            sleep(1000);
            Arm.wristUp();
            sleep(1000);
            Drivetrain.encoderTurn(130);
            sleep(1000);
            Drivetrain.encoderForward(60);
            sleep(1000);
            Drivetrain.encoderStrafe(170);
            sleep(1000);  */

        } else if (value == 2) {
            Drivetrain.encoderForward(42);
            sleep(1000);
            Drivetrain.encoderForward(-5);
            sleep(1000);
            Drivetrain.encoderStrafe(25);
            sleep(1000);
            Drivetrain.encoderForward(40);
            sleep(1000);
            Drivetrain.encoderTurn(-290); //needs to turn more here (+5 degrees)
            sleep(1000);
            Drivetrain.encoderForward(147);
           // sleep(1000);
            Drivetrain.encoderStrafe(-50);
           // sleep(1000);
            rightRobotArm.setTargetPosition(targetPosition1);
            rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRobotArm.setTargetPosition(-targetPosition1);
            leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRobotArm.setPower(0.5);
            leftRobotArm.setPower(-0.5);

            while (distance() > 4) {
                Drivetrain.encoderForward(1);
            }
            Arm.wristMid();
            sleep(1000);
            Arm.openClaw();
            sleep(1000);
            Arm.wristUp();
            sleep(1000);
            Drivetrain.encoderForward(-3);
            //Drivetrain.encoderStrafe(35);
            //sleep(1000);
            Drivetrain.encoderStrafe(32);
            Drivetrain.encoderForward(3);
            //Drivetrain.encoderForward(10);
            //sleep(1000);


        } else if (value == 3) {

            Drivetrain.encoderForward(20);
            Drivetrain.encoderHalfTurnRight(305);
            sleep(800);
            //Drivetrain.encoderForward(5);
            Drivetrain.encoderForward(-7);
            Drivetrain.encoderTurn(155);
            sleep(800);
            // Drivetrain.encoderStrafe(7);
            Drivetrain.encoderForward(47);
            sleep(800);
            Drivetrain.encoderTurn(-285);
            Drivetrain.encoderForward(122);
            //sleep(800);
            Drivetrain.encoderStrafe(-56);
            //sleep(800);
            rightRobotArm.setTargetPosition(targetPosition1);
            rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRobotArm.setTargetPosition(-targetPosition1);
            leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRobotArm.setPower(0.5);
            leftRobotArm.setPower(-0.5);

            while (distance() > 4) {
                Drivetrain.encoderForward(1);
            }
            Arm.wristMid();
            sleep(1000);
            Arm.openClaw();
            sleep(1000);
            Arm.wristUp();
            sleep(1000);
            Drivetrain.encoderForward(-5);
            //Drivetrain.encoderStrafe(35);
            //sleep(1000);
            Drivetrain.encoderStrafe(45);
            //sleep(1000);
            //Drivetrain.encoderForward();
            //sleep(1000);
        }
    }

    class ExamplePipeline extends OpenCvPipeline{
        //mk

        Mat YCbCr = new Mat();

        Mat middleCrop;
        Mat rightCrop;
        Mat leftCrop;


        double middleavgfin;

        double rightavgfin;
        double leftavgfin;

        //Mat Output = new Mat();
        Mat outPut = new Mat();
        Scalar rectColor= new Scalar(255.0, 0.0, 0.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            //    Rect middleRect = new Rect(10,1, 309, 359);
            //    Rect rightRect = new Rect(320, 1, 319, 359);
            //   Rect leftRect = new Rect(1, 1, 9, 359);
            // Rect middleRect = new Rect(190, 25, 245, 220);//good
            //   Rect rightRect = new Rect(320, 1, 319, 359);
            //test right
            // Rect rightRect = new Rect(500,70, 120, 190); //good
            //   Rect leftRect = new Rect(640, 1, 319, 359);
            // Rect leftRect = new Rect(19, 120, 90, 120); //good
            Rect middleRect = new Rect(205, 60, 260, 100);//good
            //   Rect rightRect = new Rect(320, 1, 319, 359);
            //test right
            Rect rightRect = new Rect(525, 110, 110, 168); //good
            //   Rect leftRect = new Rect(640, 1, 319, 359);
            Rect leftRect = new Rect(25, 100, 110, 168); //good


            input.copyTo(outPut);

            Imgproc.rectangle(outPut,middleRect, rectColor, 2);
            Imgproc.rectangle(outPut,rightRect, rectColor, 2);
            Imgproc.rectangle(outPut,leftRect, rectColor, 2);

            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);
            leftCrop = YCbCr.submat(leftRect);


            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(leftCrop, leftCrop, 2);

            Scalar middleavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar leftavg = Core.mean(leftCrop);


            middleavgfin = middleavg.val[0];
            rightavgfin = rightavg.val[0];
            leftavgfin = leftavg.val[0];
/*
            if((middleavgfin <= rightavgfin) && (middleavgfin > leftavgfin)){
                // if((middleavgfin < leftavgfin)){
                value = 2;
                telemetry.addLine("middle");
                telemetry.addData("middleavgfin" ,middleavg.val[0]);
                telemetry.addData("leftavgfin", leftavg.val[0]);
                telemetry.addData("rightavgfin", rightavg.val[0]);
                telemetry.addData("value2", value);
                telemetry.update();

            }
            //else if(leftavgfin < middleavgfin){

            else if((middleavgfin > leftavgfin) && ((rightavgfin >= leftavgfin) || (rightavgfin <= leftavgfin))) {
                value = 1;
                telemetry.addLine("left");
                telemetry.addData("middleavgfin", middleavg.val[0]);
                telemetry.addData("leftavgfin", leftavg.val[0]);
                telemetry.addData("rightavgfin", rightavg.val[0]);
                telemetry.addData("value1", value);
                telemetry.update();
            }

          else //if((rightavgfin < middleavgfin) && ((rightavgfin < leftavgfin))){
            {
                value = 3;
                telemetry.addLine("right");
                telemetry.addData("middleavgfin", middleavg.val[0]);
                telemetry.addData("leftavgfin", leftavg.val[0]);
                telemetry.addData("rightavgfin", rightavg.val[0]);
                telemetry.addData("value3", value);
                telemetry.update();
            }
*/
            //   if ((middleavgfin > rightavgfin) && (middleavgfin > leftavgfin) && (rightavgfin < leftavgfin)) {
            if((leftavgfin > middleavgfin) && (rightavgfin > middleavgfin)){
                //   if((middleavgfin > leftavgfin) && (middleavgfin > rightavgfin)){
                //  if(max == middleavgfin){
                value = 2;
                telemetry.addLine("middle");
                telemetry.addData("middleavgfin", middleavg.val[0]);
                telemetry.addData("leftavgfin", leftavg.val[0]);
                telemetry.addData("rightavgfin", rightavg.val[0]);
                telemetry.addData("value2", value);
                telemetry.update();


            }

            //  else {
            else  if ((middleavgfin > rightavgfin) && (rightavgfin < leftavgfin)) {
                //(middleavgfin > leftavgfin) && (rightavgfin < leftavgfin)) {
                //if ((rightavgfin > middleavgfin) && (rightavgfin < leftavgfin) && (leftavgfin > middleavgfin)) {
                // else if((rightavgfin > middleavgfin) && (rightavgfin > leftavgfin)){
                //  if(max == rightavgfin){
                value = 3;
                telemetry.addLine("right");
                telemetry.addData("middleavgfin", middleavg.val[0]);
                telemetry.addData("leftavgfin", leftavg.val[0]);
                telemetry.addData("rightavgfin", rightavg.val[0]);
                telemetry.addData("value3", value);
                telemetry.update();

            }
//   else if((leftavgfin > middleavgfin) && (leftavgfin > rightavgfin)){
            else {

                // else if ((middleavgfin > leftavgfin) && (rightavgfin > leftavgfin) && (middleavgfin > rightavgfin)) {
                // if(max == leftavgfin){
                value = 1;
                telemetry.addLine("left");
                telemetry.addData("middleavgfin", middleavg.val[0]);
                telemetry.addData("leftavgfin", leftavg.val[0]);
                telemetry.addData("rightavgfin", rightavg.val[0]);
                telemetry.addData("value1", value);
                telemetry.update();
            }


            // return(value);
            return(outPut); //mk
            // return(input);
        }

    }
    public double distance()
    {
        Dist = frontSensor.getDistance(DistanceUnit.INCH);
        return Dist;

    }
}



