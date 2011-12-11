//
//  FRC team 3681 robot program
//
//  22feb11 bv      Add joystick magnitude deadband
//  21feb11 bv      Drive speed during autonomous set to 0.3.  Less won't move.
//  21feb11 bv      Set kDeployInSpeed to -0.6 from -0.3...a little faster.  
//  18feb11 JM      Add horrible, encoderless effector code
//  13feb11 BV      Add debug code to show state of digital inputs
//  13feb11 JM      Add ramp control for deployment motor
//  03mar11 BB      Change buttons -- deploy out is 9, deploy in is 11,
//                     arm out is 5, arm in is 3
//
//
#include "WPILib.h"
#include <math.h>

/**
 * Sample mecanum drive class for FIRST 2011 Competition
 *
 */
class RobotDemo : public SimpleRobot
{
    RobotDrive *drive;              // robot drive base object
    Joystick *driveStick;           // joystick for control of motors
    Jaguar *armController;          // control arm motor
    Jaguar *deployController;       // control deployment motor
    DriverStation *ds;              // driver station object for getting selections
    DriverStationLCD *dsLCD;        // print to drive station screen
    DigitalInput *deployLimit;      // deployment full extension limit switch
    DigitalInput *undeployLimit;    // deployment minimum extension limit switch
    
    int   mDoDeployCounter;         // counter to ramp-up deployment speed
    bool  mResetDoDeployCounter;    // reset m_deployRampCounter 
    float mDeploySpeed;             // deployment speed
    
    // Output device constants
    static const UINT32 kFrontLeftMotor = 1;
    static const UINT32 kRearLeftMotor = 2;
    static const UINT32 kFrontRightMotor = 3;
    static const UINT32 kRearRightMotor = 4;
    static const UINT32 kDeploymentMotor = 5;
    static const UINT32 kArmMotor = 7;
    
    // Input device constants
    static const UINT32 kDriveStickChannel = 1;
    static const UINT32 kDeployOutButton = 9;
    static const UINT32 kDeployInButton = 11;
    static const UINT32 kArmOutButton = 5;
    static const UINT32 kArmInButton = 3;
    static const float kDeployOutSpeed = 1.0;
    static const float kDeployInSpeed = -0.5;
    static const float kArmOutSpeed = 1.0;
    static const float kArmInSpeed = -1.0;
    
    // Autonomous state machine
    typedef enum {kAutoStateForward, kAutoStateStop, kAutoStateGrip, kAutoStateDone} AutoState;

public:
    /*
     * RobotDemo constructor
     * This code creates instances of the objects and sets up the driving directions
     * for the RobotDrive object. 
     */
    RobotDemo() {
        drive = new RobotDrive(kFrontLeftMotor, kRearLeftMotor, 
                               kFrontRightMotor, kRearRightMotor);
        /* if needed, uncomment these
        drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
        drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
        drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
        drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
        */
        driveStick = new Joystick(kDriveStickChannel);
        deployController = new Jaguar(kDeploymentMotor);
        armController = new Jaguar(kArmMotor);
        ds = DriverStation::GetInstance();  // driver station instance for digital I/O
        dsLCD = DriverStationLCD::GetInstance();
        mDoDeployCounter = 0;
        mResetDoDeployCounter = true;
        mDeploySpeed = 0.0;
    }
    
    /**
     * Clean up after yourself...
     */
    ~RobotDemo() {
        delete ds;
        delete driveStick;
        delete deployController;
        delete armController;
        delete drive;
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    void Autonomous() {
        const float driveSpeed = 0.3;
        const float distanceCalibration = 10.0; // in feet???
        const float driveTime = driveSpeed * distanceCalibration;
        
        const float stopTime = 0.5; // pause after stopping forward driving
        const float armTime = 0.0;  // don't do anything with the arm for now...
        
        AutoState driveState = kAutoStateForward;
        
        double startTime = GetTime();
        drive->SetSafetyEnabled(true);
        
        while(IsAutonomous()) {
            double currentTime = GetTime();
            double interval = currentTime - startTime;
            
            switch(driveState) {
                case kAutoStateForward:
                    if (interval > driveTime) {
                        driveState = kAutoStateStop;
                        startTime = GetTime();
                    }
                    else {
                        drive->MecanumDrive_Polar(driveSpeed, 0.0, 0.0);
                    }
                    break;
                case kAutoStateStop:
                    if (interval > stopTime) {
                        driveState = kAutoStateGrip;
                        startTime = GetTime();
                    }
                    else {
                        drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
                    }
                    break;
                case kAutoStateGrip:
                    if (interval > armTime) {
                        driveState = kAutoStateDone;
                    }
                    else {
                        armController->SetSpeed(kArmOutSpeed);
                    }
                    break;
                case kAutoStateDone:
                    armController->SetSpeed(0.0);
                    break;
            }
            Wait(0.01);
        }   
    }

    /**
     * Uses the "twist" tab to propotionally trim the Z-axis rotation
     */
    float getTrimmedZ() {
        float rotation = driveStick->GetZ();    // values -1.0 to 1.0
        float twist = (driveStick->GetTwist() + 1.0) / 2.0; // scale values from 0.0 to 1.0
        float trimmedZ = twist * rotation * rotation;
        if (rotation >= 0.0) {
            return trimmedZ;
        }
        else {
            return -trimmedZ;
        }
    }
    
    /**
     * drive motors with driveStick
     **/
    void doDriveControl() {
        float magnitude = driveStick->GetMagnitude();
        float direction = driveStick->GetDirectionDegrees();
        float rotation = getTrimmedZ();
        
        // Magnitude deadband to avoid motor current at low joystick angle
        if (fabs(magnitude) < 0.1) magnitude = 0.0;
        
        dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "mg: %f", magnitude);
        dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "dr: %f", direction);
        dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "rt: %f", rotation);
        dsLCD->UpdateLCD();
        drive->MecanumDrive_Polar(magnitude, direction, rotation);      
    }
    
    /**
     * control effector motors via effectorStick and encoders
     **/
    void doEffectorControl () {
        if (driveStick->GetRawButton(kArmOutButton)) {
            armController->SetSpeed(kArmOutSpeed);
        }
        else if (driveStick->GetRawButton(kArmInButton)) {
            armController->SetSpeed(kArmInSpeed);
        }
        else {
            armController->SetSpeed(0.0);
        }
    }
    
    /**
     * control minibot deployment via effectorStick
     * */
    void doDeployControl () {
        // when deploying out, ramp up speed by scaling by number of invocations of doDeployControl
        // up to 100
        if (driveStick->GetRawButton(kDeployOutButton)) {
            if (mResetDoDeployCounter) {
                mDoDeployCounter = 0;
                mResetDoDeployCounter = false;
            }
            if (mDoDeployCounter < 20) {
                mDoDeployCounter++;
            }
            mDeploySpeed = mDoDeployCounter * kDeployOutSpeed / 20;
            deployController->SetSpeed(mDeploySpeed);
        }
        else {
            mResetDoDeployCounter = true;
            
            if (driveStick->GetRawButton(kDeployInButton)) {
                deployController->SetSpeed(kDeployInSpeed);
            }
            else {
                deployController->SetSpeed(0.0);
            }
        }
    }
    
    /**
     * This function is called once each time the robot enters operator control.
     */     
    void OperatorControl() {
        drive->SetSafetyEnabled(true);
        while (IsOperatorControl())
        {
            doDriveControl();
            doEffectorControl();
            doDeployControl();
            Wait(0.01);             // wait for a motor update time
        }
    }
};

START_ROBOT_CLASS(RobotDemo);

