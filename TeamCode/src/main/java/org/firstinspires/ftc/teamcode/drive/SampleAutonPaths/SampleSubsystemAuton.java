package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class SampleSubsystemAuton extends LinearOpMode {
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0, 0, 0);

    public static double kV = 0.00071;
    public static double kA = 0;
    public static double kStatic = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareFile drive = new HardwareFile(hardwareMap);
        //for(double i=0.6;i<0.8;i+=0.05) {
            drive.tilter.setPosition(0.715);
            sleep(5000);
           // telemetry.addData("Pos",i);
            //telemetry.update();
        //}
    }

}
