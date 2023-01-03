package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Constant;

public class Hardware {


    public static DcMotorEx motorFl;
    public static DcMotorEx motorBl;
    public static DcMotorEx motorBr;
    public static DcMotorEx motorFr;

    public static DcMotorEx leftEncoder;
    public static DcMotorEx rightEncoder;
    public static DcMotorEx frontEncoder;
    public static DcMotorEx slider1;
    public static DcMotorEx slider2;

    /*public static Servo intakeClaw;
    public static Servo moveIntake;*/
    public static Servo mainClaw;
    public static Servo rotateClaw;
    public static Servo leftArm;
    public static Servo rightArm;
    public static Servo inOutAngle;
    //public static Servo intake;


    //public static RevColorSensorV3 intakeSensor1;
    public static RevColorSensorV3 clawSensor;

    /*public static RevBlinkinLedDriver ledBand;

    public static LynxModule expansionHub;*/

    public static DigitalChannel limitSwitch1;
    public static DigitalChannel limitSwitch2;

    HardwareMap hwMap           =  null;

    public Hardware(){
    }

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        motorFl = hwMap.get(DcMotorEx.class, "frontLeft");
        motorBl = hwMap.get(DcMotorEx.class, "backLeft");
        motorBr = hwMap.get(DcMotorEx.class, "backRight");
        motorFr = hwMap.get(DcMotorEx.class, "frontRight");

        leftEncoder = hwMap.get(DcMotorEx.class, "backRight");
        rightEncoder = hwMap.get(DcMotorEx.class, "frontRight");
        frontEncoder = hwMap.get(DcMotorEx.class, "frontLeft");
        slider1 = hwMap.get(DcMotorEx.class, "sliderL");
        slider2 = hwMap.get(DcMotorEx.class, "sliderR");

        motorFl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFl.setPower(0.0);
        motorBl.setPower(0.0);
        motorBr.setPower(0.0);
        motorFr.setPower(0.0);
        slider1.setPower(0.0);
        slider2.setPower(0.0);

        motorFl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBr.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFr.setDirection(DcMotorSimple.Direction.FORWARD);
        slider1.setDirection(DcMotorSimple.Direction.FORWARD);
        slider2.setDirection(DcMotorSimple.Direction.REVERSE);

        mainClaw   = hwMap.get(Servo.class, "mainClaw");
        rotateClaw = hwMap.get(Servo.class, "rotateClaw");
        leftArm = hwMap.get(Servo.class, "leftArm");
        rightArm = hwMap.get(Servo.class, "rightArm");
        inOutAngle = hwMap.get(Servo.class, "inOutAngle");/*
        intakeClaw = hwMap.get(Servo.class, "intakeClaw");
        moveIntake = hwMap.get(Servo.class, "moveIntake");
        intake     = hwMap.get(Servo.class, "intake");
*/
        mainClaw  .setPosition(Constant.closedClaw);
        rotateClaw.setPosition(Constant.verticalCone);
        leftArm.setPosition(Constant.leftArmCollect);
        rightArm.setPosition(Constant.rightArmCollect);
        inOutAngle.setPosition(Constant.in);/*
        intakeClaw.setPosition(constant.openIntake);
        moveIntake.setPosition(constant.intakeCollect);
        intake    .setPosition(constant.intakeStart);

        intakeSensor1 = hwMap.get(RevColorSensorV3.class, "intakeSensor1");*/
        clawSensor    = hwMap.get(RevColorSensorV3.class, "clawSensor");
       /*
        ledBand = hwMap.get(RevBlinkinLedDriver.class, "ledBand");
        ledBand.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);*/

        limitSwitch1 = hwMap.get(DigitalChannel.class, "limitSwitch1");
        limitSwitch2 = hwMap.get(DigitalChannel.class, "limitSwitch2");
    }
}
