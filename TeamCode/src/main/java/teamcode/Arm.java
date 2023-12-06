package teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public static DcMotorEx leftRobotArm;
    public static DcMotorEx rightRobotArm;
    public static Servo rightClaw;
    public static Servo leftClaw;
    public static Servo wrist;


    public Arm() {

    }

    public static void init(DcMotorEx leftRobotArm, DcMotorEx rightRobotArm, Servo rightrightClaw, Servo leftrightClaw, Servo wrist) {
        Arm.leftRobotArm = leftRobotArm;
        Arm.rightRobotArm = rightRobotArm;
        Arm.rightClaw = rightrightClaw;
        Arm.leftClaw = leftrightClaw;
        Arm.wrist = wrist;

        leftRobotArm.setDirection(DcMotorEx.Direction.REVERSE);
        rightrightClaw.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        leftRobotArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRobotArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftRobotArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRobotArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        leftRobotArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRobotArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public static void openClaw() {
        rightClaw.setPosition(0.3);
        leftClaw.setPosition(0.1);
    }

    public static void closeClaw() {
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
    }
    public static void wristLow(){
        wrist.setPosition(0.31);
    }
    public static void wristMid(){
        wrist.setPosition(0.40);
    }
    public static void wristUp(){
        wrist.setPosition(1.2);
    }

    public static void armFloor() {
        leftRobotArm.setTargetPosition(10);
        rightRobotArm.setTargetPosition(10);

        leftRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftRobotArm.setPower(0.5);
        rightRobotArm.setPower(0.5);
    }

    public static void armLow() {
        leftRobotArm.setTargetPosition(-10);
        rightRobotArm.setTargetPosition(10);

        leftRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftRobotArm.setPower(0.5);
        rightRobotArm.setPower(0.5);
    }

    public static void armMedium() {
        leftRobotArm.setTargetPosition(-300);
        rightRobotArm.setTargetPosition(300);

        leftRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftRobotArm.setPower(0.5);
        rightRobotArm.setPower(0.5);
    }

    public static void armHigh() {
        leftRobotArm.setTargetPosition(-480);
        rightRobotArm.setTargetPosition(480);

        leftRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightRobotArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftRobotArm.setPower(0.5);
        rightRobotArm.setPower(0.5);
    }
}


