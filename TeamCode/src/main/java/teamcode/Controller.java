


package teamcode;//imports needed for ftc robot code.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="TeleOp", group="Android Studio")
public class Controller extends LinearOpMode {
    // Declare every variable being used in the program here.
    private ElapsedTime runtime = new ElapsedTime();
    public static DcMotorEx leftFront;
    public static DcMotorEx rightFront;
    public static DcMotorEx leftRear;
    public static DcMotorEx rightRear;
    public static DcMotorEx leftRobotArm;
    public static DcMotorEx rightRobotArm;
    public static DcMotorEx armExtender;
    public static Servo rightClaw;
    public static Servo leftClaw;
    public static Servo wrist;
    public static Servo droneLauncher;
    public static Servo droneHolder;
    int targetPosition1 = 10;
    int targetPosition2 = 328;
    int targetPosition3 = 480;
    int targetPosition4 = 200;
    int targetPosition5 = 560;





    @Override
    public void runOpMode() throws InterruptedException {
        //28 counts/tics per revolution
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftRobotArm = hardwareMap.get(DcMotorEx.class, "leftRobotArm");
        rightRobotArm = hardwareMap.get(DcMotorEx.class, "rightRobotArm");
        armExtender = hardwareMap.get(DcMotorEx.class, "armExtender");
        leftRobotArm = hardwareMap.get(DcMotorEx.class, "leftRobotArm");
        rightRobotArm = hardwareMap.get(DcMotorEx.class, "rightRobotArm");
        armExtender = hardwareMap.get(DcMotorEx.class, "armExtender");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        droneHolder = hardwareMap.get(Servo.class, "droneHolder");
        wrist = hardwareMap.get(Servo.class, "wrist");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        leftRobotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRobotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRobotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRobotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightClaw.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);



        Drivetrain.init(leftFront, rightFront, leftRear, rightRear);
        Arm.init(rightRobotArm, leftRobotArm, rightClaw,leftClaw, wrist);

        droneLauncher.setPosition(0.6);


        waitForStart();

        while (opModeIsActive()) {
            leftFront.setPower((-gamepad1.left_stick_y/1.24) + (gamepad1.right_stick_x/1.24) + (gamepad1.left_stick_x/1.24));
            rightFront.setPower((-gamepad1.left_stick_y/1.24) + (-gamepad1.right_stick_x/1.24) + (-gamepad1.left_stick_x/1.24));
            leftRear.setPower((-gamepad1.left_stick_y/1.24) + (-gamepad1.right_stick_x/1.24) + (gamepad1.left_stick_x/1.24));
            rightRear.setPower((-gamepad1.left_stick_y/1.24) + (gamepad1.right_stick_x/1.24) + (-gamepad1.left_stick_x/1.24));


            if (gamepad2.dpad_right) {
                rightRobotArm.setTargetPosition(targetPosition2);
                rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRobotArm.setPower(0.5);
                leftRobotArm.setTargetPosition(-targetPosition2);
                leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRobotArm.setPower(-0.5);
            }

            if (gamepad2.dpad_down) {
                rightRobotArm.setTargetPosition(targetPosition1);
                rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRobotArm.setPower(0.5);
                leftRobotArm.setTargetPosition(-targetPosition1);
                leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRobotArm.setPower(-0.5);
            }

            if (gamepad2.dpad_up) {
                rightRobotArm.setTargetPosition(targetPosition3);
                rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRobotArm.setPower(0.5);
                leftRobotArm.setTargetPosition(-targetPosition3);
                leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRobotArm.setPower(-0.5);
                wrist.setPosition(0.9);
            }

            if (gamepad2.dpad_left) {
                rightRobotArm.setTargetPosition(targetPosition4);
                rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRobotArm.setPower(0.5);
                leftRobotArm.setTargetPosition(-targetPosition4);
                leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRobotArm.setPower(-0.5);
            }

            if (gamepad2.share) {
                rightRobotArm.setTargetPosition(targetPosition5);
                rightRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRobotArm.setPower(0.5);
                leftRobotArm.setTargetPosition(-targetPosition5);
                leftRobotArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRobotArm.setPower(-0.5);
                wrist.setPosition(0.9);
            }



            armExtender.setPower(gamepad2.right_stick_y);
            if (gamepad2.circle) {
                leftClaw.setPosition(0);
            } else if(gamepad2.triangle) {
                leftClaw.setPosition(0.1);
            }

            if (gamepad2.circle) {
                rightClaw.setPosition(0);
            } else if(gamepad2.triangle) {
                rightClaw.setPosition(0.3);
            }

            if (gamepad2.left_bumper) {
                wrist.setPosition(0.37);
            }
            else if (gamepad2.right_bumper) {
                wrist.setPosition(1.2);
            }

            if (gamepad1.circle) {
                droneHolder.setPosition(1);
            }
            else if (gamepad1.right_bumper) {
                droneHolder.setPosition(0);
            }

            if (gamepad1.triangle) {
                droneLauncher.setPosition(0.6);
            }
            else if (gamepad1.square) {
                droneLauncher.setPosition(0);
            }

            telemetry.addData("velocityLeft", leftRobotArm.getVelocity());
            telemetry.addData("positionLeft", leftRobotArm.getCurrentPosition());
            telemetry.addData("is at targetLeft", !leftRobotArm.isBusy());
            telemetry.update();







            //Gamepad1 sticks correlate to different functions, like strafing, turning and going forward.
            //Gamepad2 sticks control the arm and the arm extender. The buttons control the claw.


        }

    }
}

