package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DriverControlJr", group="Linear Opmode")
public class TeleCode extends LinearOpMode implements Runnable{
    public static HardwareFile robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareFile(hardwareMap);
        //robot.driveTrain=new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            run();
        }
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    public void run() {
        /*robot.driveTrain.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * lyMult,
                        -gamepad1.left_stick_x * lxMult,
                        -gamepad1.right_stick_x * 0.92 * rxMult
                )
        );*/
        //setMultiplier();
        //INTAKE
        if(gamepad2.right_bumper) {
            robot.intakeMotor.setPower(-0.6);
            robot.transportMotor.setPower(0.6);
        }else if(gamepad2.right_trigger==1){
            robot.intakeMotor.setPower(0.6);
            robot.transportMotor.setPower(-0.6);
        }else{
            robot.intakeMotor.setPower(0);
            robot.transportMotor.setPower(0);
        }
        //WOBBLE
        if(gamepad2.a){
            angleToPosition(0);
//            sleep(500);
//            robot.grabberWobble.setPosition(0.63);
        }else if(gamepad2.y){
//            robot.grabberWobble.setPosition(90);
//            sleep(500);
            angleToPosition(90);
        }
        //SHOOTER
       if(gamepad2.right_trigger > 0){
//            robot.shooter.setPower(1);
//            sleep(500);
//            for(int i=0;i<=3;++i){
                robot.shooterpush.setPosition(0.4);
//                sleep(100);
//                robot.shooterpush.setPosition(0.85);
//                sleep(1500);
//            }
//            robot.shooter.setPower(0);
        }
        if(gamepad2.left_trigger > 0){
//            robot.shooter.setPower(0.4);
//            sleep(500);
//            for(int i=0;i<1;++i){
//                sleep(2500);
//                robot.shooterpush.setPosition(0.4);
//                sleep(100);
                robot.shooterpush.setPosition(0.85);
//                sleep(1500);
//            }
//            robot.shooter.setPower(0);
        }
        telemetry.update();
//        robot.driveTrain.update();
    }

    private void setMultiplier() {
        //MULTIPLIERS
        if (gamepad1.left_trigger >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        }
        else {
            lxMult = 1;
            rxMult = 1;
            lyMult = 1;
        }

        if (gamepad1.right_bumper) {
            lxMult = -lxMult;
            lyMult = -lyMult;
        }
    }
    public void angleToPosition(double angle){
        int target = (int)((angle/360)*537);
        robot.armWobble.setTargetPosition(target);
        robot.armWobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armWobble.setPower(0.25);

        while (robot.armWobble.getCurrentPosition() != target) {
        }
        robot.armWobble.setPower(0);
    }
}
