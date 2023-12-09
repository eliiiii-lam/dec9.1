package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDF_ARM extends OpMode {

    private PIDController controller;

    public static double p=0, i=0,d=0;

    public static double f=0;

    public static int target = 0;

    private final double ticks_in_degree = 5281.1/180.0;

    private DcMotor elbow;

    private DcMotor elbow2;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elbow = hardwareMap.dcMotor.get("elbow");
        elbow = hardwareMap.dcMotor.get("elbow2");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        int armpos = elbow.getCurrentPosition();

        double pid = controller.calculate(armpos,target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        elbow.setPower(power);

        telemetry.addData("pos", armpos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}
