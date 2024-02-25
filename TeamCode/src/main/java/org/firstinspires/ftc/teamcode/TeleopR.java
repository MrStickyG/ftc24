/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp

public class TeleopR extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRMotor = null;
    private DcMotor FLMotor = null;
    private DcMotor BRMotor;
    private DcMotor BLMotor;
    private DcMotor Lift;
    private DcMotor Climber;

    private DcMotor intakeMotor;

    private Servo wristServo;
    private Servo shooter;
    private Servo dispenser;

    int safePos;
    int max=1500;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FRMotor  = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        FLMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        Lift = hardwareMap.get(DcMotorEx.class, "Lift");
        Climber = hardwareMap.get(DcMotorEx.class, "climb");

        Lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Climber.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Climber.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Climber.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //setMode(DcMotor.RunMode.RUN_TO_POSITION);


        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        wristServo = hardwareMap.get(Servo.class, "wrist");
        dispenser = hardwareMap.get(Servo.class, "dispenser");
        shooter = hardwareMap.get(Servo.class, "shoot");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        Climber.setDirection(DcMotorEx.Direction.REVERSE);
        Lift.setDirection(DcMotorEx.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        dispenser.setPosition(.75);
        shooter.setPosition(.8);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double BLpower;
            double FLpower;
            double FRpower;
            double BRpower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double turn = gamepad1.left_stick_x;
            double drive  =  -gamepad1.left_stick_y;
            double rotate = -gamepad1.right_stick_x;
            boolean mSlideU=gamepad2.y;
            boolean mSlideD=gamepad2.a;

            double DConstant=1;
            boolean up =gamepad1.y;
            boolean down =gamepad1.a;
            boolean shoot=gamepad1.x && gamepad1.b;
            boolean intake = gamepad1.left_bumper;

            if(Lift.getCurrentPosition()<-2250){
                wristServo.setPosition(.90);
                DConstant=.3;
            }else{
                wristServo.setPosition(.29);
                DConstant=1;
            }
            BLpower = DConstant*Range.clip(-drive - turn +rotate, -1.0, 1.0) ;
            FLpower = DConstant*Range.clip(drive - turn - rotate, -1.0, 1.0) ;
            FRpower = DConstant*Range.clip(drive + turn + rotate, -1.0, 1.0) ;
            BRpower = DConstant*Range.clip(-drive + turn - rotate, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            if(shoot){
                shooter.setPosition(0);
            }


            if(gamepad2.left_bumper){
                slide(0);
                //wristServo.setPosition(0);
                //dispenser.setPosition(.8);
            }
            if(gamepad2.a){
                dispense();
            }
            if(gamepad2.right_bumper){
                slide(-3200);
            }
            if (gamepad2.right_trigger>0){
                //climberdown();
                intakeMotor.setPower(.53);
                //dispense();
            }else if(gamepad2.left_trigger>0){
                intakeMotor.setPower(-.75);
            }else{
                intakeMotor.setPower(0);
            }
            if(up){
                climberup();

            }
            if(down){
                climberdown();

            }
            /*if(mSlideU){
                slide(1);
            }else if(mSlideD){
                slide(-1);
            }else{slide(0);}*/

            // Send calculated power to wheels
            FLMotor.setPower(FLpower);
            BLMotor.setPower(BLpower);
            BRMotor.setPower(BRpower);
            FRMotor.setPower(FRpower);
            //slide(mSlideU-mSlideD);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "SlidePos"+ Lift.getCurrentPosition());
            telemetry.addData("Status", "climPos"+ Climber.getCurrentPosition());

            telemetry.addData("Status", "WristPos"+ wristServo.getPosition());
            telemetry.addData("Status", "DispensePos"+ dispenser.getPosition());
            telemetry.addData("Status", "Safe"+ LiftSafe());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }

    }
    public void dispense() {
        if(Lift.getCurrentPosition()<1500){
            dispenser.setPosition(0.55);
            sleep(400);
            dispenser.setPosition(.72);
        }
    }
   //control slides
    public void slide(int pos){
        if(true){
            Lift.setTargetPosition(pos);
            Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Lift.setPower(1);
        }


    }
    public boolean LiftSafe(){
        if(Lift.getCurrentPosition()>=0&&Lift.getCurrentPosition()<=max){
            return true;
        }else{
            return false;
        }

    }
    public void climberup(){
        if(true) {
            Climber.setPower(1);
            Climber.setTargetPosition(-18000);
            Climber.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        }
    }
    public void climberdown(){
        if (true) {
            Climber.setPower(1);
            Climber.setTargetPosition(-1000);
            Climber.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }


    }
}
