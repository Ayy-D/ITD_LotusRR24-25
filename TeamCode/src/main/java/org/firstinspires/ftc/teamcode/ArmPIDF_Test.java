package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.leftController.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp

public class ArmPIDF_Test extends OpMode{

    public static double p = 0, i = 0, d = 0, f = 0;
    public PIDFController leftController;
    public PIDFController rightController;
    public static int target = 0;
    private final double ticks_in_degree = 1.4936;

    private DcMotorEx armRight = null;
    private DcMotorEx armLeft = null;

    private DcMotorEx sRight = null;
    private DcMotorEx sLeft = null;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init(){

        leftController = new PIDFController(p, i, d, f);
        rightController = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armLeft = hardwareMap.get(DcMotorEx.class, "rotateL");
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armLeft.setDirection(DcMotor.Direction.REVERSE);

        armRight = hardwareMap.get(DcMotorEx.class, "rotateR");
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setDirection(DcMotor.Direction.REVERSE);

        sLeft = hardwareMap.get(DcMotorEx.class, "slideL"); // The right slide
        sLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sRight = hardwareMap.get(DcMotorEx.class, "slideR"); // The left slide
        sRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sRight.setDirection(DcMotor.Direction.REVERSE);
    }


    public void loop(){
        leftController.setPIDF(p, i, d, f);
        leftController.setTolerance(3,10);
        rightController.setPIDF(p, i, d, f);
        rightController.setTolerance(3,10);
        int armPosL = armLeft.getCurrentPosition();
        int armPosR = armRight.getCurrentPosition();

        double pidLeft = leftController.calculate(armPosL, target);
        double pidRight = rightController.calculate(armPosR, target);
        //double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        //double powerL = pidLeft + ff;
        //double powerR = pidRight + ff;

        armLeft.setVelocity(pidLeft);
        armRight.setVelocity(pidRight);

        sLeft.setPower(0.8);
        sLeft.setTargetPosition(900);
        sLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sRight.setPower(0.8);
        sRight.setTargetPosition(900);
        sRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Arm Right Position", armPosR);
        telemetry.addData("Arm Left Position", armPosL);
        telemetry.update();


    }



}
