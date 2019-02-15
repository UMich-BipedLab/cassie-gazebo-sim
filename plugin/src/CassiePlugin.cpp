/*
 * Copyright (c) 2018 Agility Robotics
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "CassiePlugin.hpp"
#include <cmath>
// For reading keyboard
// #include "/opt/ros/kinetic/include/std_msgs/Int16.h"
#include <stdio.h>
#include <stdlib.h>

CassiePlugin::CassiePlugin() :
    kMotorBits_{13, 13, 13, 13, 18, 13, 13, 13, 13, 18},
    kGearRatio_{25, 25, 16, 16, 50, 25, 25, 16, 16, 50},
    kMaxSpeed_{2900, 2900, 1300, 1300, 5500, 2900, 2900, 1300, 1300, 5500},
    kMaxTorque_{4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2, 0.9},
    kMotorOffset_{0, 0, 0, -0.785398, 0, 0, 0, 0, -0.785398, 0},
    kJointBits_{18, 18, 13, 18, 18, 13},
    kJointOffset_{0, 1.012291, 0, 0, 1.01229, 0},
    motorFilterB_{2727, 534, -2658, -795, 72, 110, 19, -6, -3},
    jointFilterB_{12.348, 12.348, -12.348, -12.348},
    jointFilterA_{1.0, -1.7658, 0.79045},
    cassieOut_{},
    headerInfo_{},
    cassieUserIn_{},
    Kp_PD_{400, 200, 500, 500, 10, 400, 200, 500, 500, 10},
    Kd_PD_{10, 4, 15, 15, 2, 10, 4, 15, 15, 2},
    u_cont_beta_{50},
    torque_discontinuity_threshold_{50},
    u_cont_t0_{0},
    torques_unmodified_prev_{0},
    u_cont_alpha_{0}
{
    // Pointers to each drive struct in cassieOut
    driveOut_ = {
        &cassieOut_.leftLeg.hipRollDrive,
        &cassieOut_.leftLeg.hipYawDrive,
        &cassieOut_.leftLeg.hipPitchDrive,
        &cassieOut_.leftLeg.kneeDrive,
        &cassieOut_.leftLeg.footDrive,
        &cassieOut_.rightLeg.hipRollDrive,
        &cassieOut_.rightLeg.hipYawDrive,
        &cassieOut_.rightLeg.hipPitchDrive,
        &cassieOut_.rightLeg.kneeDrive,
        &cassieOut_.rightLeg.footDrive
    };

    // Pointers to each joint struct in cassieOut
    jointOut_ = {
        &cassieOut_.leftLeg.shinJoint,
        &cassieOut_.leftLeg.tarsusJoint,
        &cassieOut_.leftLeg.footJoint,
        &cassieOut_.rightLeg.shinJoint,
        &cassieOut_.rightLeg.tarsusJoint,
        &cassieOut_.rightLeg.footJoint
    };

    // Initialize cassieOut
    initCassieOut(&cassieOut_);

    // Create cassie core
    corePtr_ = cassie_core_sim_alloc();
    cassie_core_sim_setup(corePtr_);

    // Separate UDP input/output buffers into header and payload
    headerInPtr_ = recvBuf_;
    dataInPtr_ = &recvBuf_[PACKET_HEADER_LEN];
    headerOutPtr_ = sendBuf_;
    dataOutPtr_ = &sendBuf_[PACKET_HEADER_LEN];

    memset(&slrt_data_prev_, 0, sizeof (cassie_slrt_data_t));

    runSim_ = false;
}


CassiePlugin::~CassiePlugin()
{
}


void CassiePlugin::setMotorEncoder(elmo_out_t *drive, double position,
                                   motor_filter_t *filter, const int bits)
{
    // Position
    // Get digital encoder value
    int encoderValue = position / (2 * M_PI) * (1 << bits);
    double scale     = (2 * M_PI) / (1 << bits);
    drive->position  = encoderValue * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero  = true;
    for (size_t i = 0; i < MOTOR_FILTER_NB; ++i)
        allzero  &= filter->x[i] == 0;
    if (allzero) {
        // If all filter values are zero, initialize the signal array
        // with the current encoder value
        for (size_t i = 0; i < MOTOR_FILTER_NB; ++i)
            filter->x[i] = encoderValue;
    }

    // Shift and update unfiltered signal array
    for (size_t i = MOTOR_FILTER_NB - 1; i > 0; --i)
        filter->x[i] = filter->x[i - 1];
    filter->x[0] = encoderValue;

    // Compute filter value
    int y = 0;
    for (size_t i = 0; i < MOTOR_FILTER_NB; ++i)
        y += filter->x[i] * motorFilterB_[i];
    drive->velocity = y * scale / M_PI;
}


void CassiePlugin::setJointEncoder(cassie_joint_out_t *joint, double position,
                                   joint_filter_t *filter, const int bits)
{
    // Position
    // Get digital encoder value
    int encoderValue = position / (2 * M_PI) * (1 << bits);
    double scale     = (2 * M_PI) / (1 << bits);
    joint->position  = encoderValue * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero  = true;
    for (size_t i = 0; i < JOINT_FILTER_NB; ++i)
        allzero &= filter->x[i] == 0;
    if (allzero) {
        // If all filter values are zero, initialize the signal array
        // with the current encoder value
        for (size_t i = 0; i < JOINT_FILTER_NB; ++i)
            filter->x[i] = joint->position;
    }

    // Shift and update signal arrays
    for (size_t i = JOINT_FILTER_NB - 1; i > 0; --i)
        filter->x[i] = filter->x[i - 1];
    filter->x[0] = joint->position;
    for (size_t i = JOINT_FILTER_NA - 1; i > 0; --i)
        filter->y[i] = filter->y[i - 1];

    // Compute filter value
    filter->y[0] = 0;
    for (size_t i = 0; i < JOINT_FILTER_NB; ++i)
        filter->y[0] += filter->x[i] * jointFilterB_[i];
    for (size_t i = 1; i < JOINT_FILTER_NA; ++i)
        filter->y[0] -= filter->y[i] * jointFilterA_[i];
    joint->velocity = filter->y[0];
}


double CassiePlugin::setMotor(gazebo::physics::JointPtr outjoint, double u,
                              const bool sto, const double ratio,
                              const double tmax, const double wmax)
{
    // Get rotor velocity
    double w = outjoint->GetVelocity(0) * ratio;

    // Calculate torque limit based on motor speed
    double tlim = 2 * tmax * (1 - fabs(w) / wmax);
    tlim = fmax(fmin(tlim, tmax), 0);

    // Apply STO
    if (sto)
        u = 0;

    // Compute output-side torque
    double tau = ratio * copysign(fmin(fabs(u / ratio), tlim), u);
    outjoint->SetForce(0, tau);

    // Return limited output-side torque
    return tau;
}


void CassiePlugin::initElmoOut(elmo_out_t *elmoOut, double torqueLimit,
                               const double gearRatio)
{
    elmoOut->statusWord       = 0x0637;
    elmoOut->dcLinkVoltage    = 48;
    elmoOut->driveTemperature = 30;
    elmoOut->torqueLimit      = torqueLimit;
    elmoOut->gearRatio        = gearRatio;
}

void CassiePlugin::initCassieLegOut(cassie_leg_out_t *legOut)
{
    legOut->medullaCounter = 1;
    legOut->medullaCpuLoad = 94;
    initElmoOut(&legOut->hipRollDrive,  140.63, 25);
    initElmoOut(&legOut->hipYawDrive,   140.63, 25);
    initElmoOut(&legOut->hipPitchDrive, 216.16, 16);
    initElmoOut(&legOut->kneeDrive,     216.16, 16);
    initElmoOut(&legOut->footDrive,      45.14, 50);
}


void CassiePlugin::initCassieOut(cassie_out_t *cassieOut)
{
    // Zero-initialize the struct
    *cassieOut = {};

    // Calibrated
    cassieOut->isCalibrated = true;

    // Pelvis
    cassieOut->pelvis.medullaCounter = 1;
    cassieOut->pelvis.medullaCpuLoad = 159;
    cassieOut->pelvis.vtmTemperature = 40;

    // Target PC
    cassieOut->pelvis.targetPc.etherCatStatus[1] = 8;
    cassieOut->pelvis.targetPc.etherCatStatus[4] = 1;
    cassieOut->pelvis.targetPc.taskExecutionTime = 2e-4;
    cassieOut->pelvis.targetPc.cpuTemperature    = 60;

    // Battery
    cassieOut->pelvis.battery.dataGood = true;
    cassieOut->pelvis.battery.stateOfCharge = 1;
    for (size_t i = 0; i < 4; ++i)
        cassieOut->pelvis.battery.temperature[i] = 30;
    for (size_t i = 0; i < 12; ++i)
        cassieOut->pelvis.battery.voltage[i] = 4.2;

    // Radio
    cassieOut->pelvis.radio.radioReceiverSignalGood = true;
    cassieOut->pelvis.radio.receiverMedullaSignalGood = true;
    cassieOut->pelvis.radio.channel[8] = 1;

    // VectorNav
    cassieOut->pelvis.vectorNav.dataGood = true;
    cassieOut->pelvis.vectorNav.pressure = 101.325;
    cassieOut->pelvis.vectorNav.temperature = 25;

    // Legs
    initCassieLegOut(&cassieOut->leftLeg);
    initCassieLegOut(&cassieOut->rightLeg);
}


void CassiePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Store a pointer to the world
    this->worldPtr_ = model->GetWorld();
    // Store pointers to each model joint corresponding to a motor output
    motor_ = {
        model->GetJoint("left-roll-op"),
        model->GetJoint("left-yaw-op"),
        model->GetJoint("left-pitch-op"),
        model->GetJoint("left-knee-op"),
        model->GetJoint("left-foot-op"),
        model->GetJoint("right-roll-op"),
        model->GetJoint("right-yaw-op"),
        model->GetJoint("right-pitch-op"),
        model->GetJoint("right-knee-op"),
        model->GetJoint("right-foot-op")
    };

    // Store pointers to each model joint corresponding to a measured joint
    joint_ = {
        model->GetJoint("left-knee-shin-joint"),
        model->GetJoint("left-shin-tarsus-joint"),
        model->GetJoint("left-foot-op"),
        model->GetJoint("right-knee-shin-joint"),
        model->GetJoint("right-shin-tarsus-joint"),
        model->GetJoint("right-foot-op")
    };

    // Store a pointer to the pelvis link
    pelvisPtr_ = model->GetLink("pelvis");

    // Update Rate
    this->updateRate_ = 2000;
    this->updatePeriod_ = 1.0 / this->updateRate_;
    lastUpdateTime_ = this->worldPtr_->SimTime();

    // Open UDP socket
    sock_ = udp_init_host("0.0.0.0", "25000");

    // Listen to the update event which is broadcast every simulation iteration
    this->updateConnectionPtr_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&CassiePlugin::onUpdate, this));

    // Start thread
    std::cout<<"start\n";
    std::thread inthread(&KeyboardInput::getInput, &keyboard_);
    inthread.detach();
    std::cout<<"end\n";

}


void CassiePlugin::Reset() 
{
    // Fix pelvis to world
    auto model = this->worldPtr_->ModelByName("cassie");
    gazebo::physics::JointPtr x_joint = model->GetJoint("x");
    gazebo::physics::JointPtr y_joint = model->GetJoint("y");
    gazebo::physics::JointPtr z_joint = model->GetJoint("z");
    gazebo::physics::JointPtr yaw_joint = model->GetJoint("yaw");
    gazebo::physics::JointPtr pitch_joint = model->GetJoint("pitch");
    x_joint->SetLowerLimit(0,0); 
    x_joint->SetUpperLimit(0,0); 
    y_joint->SetLowerLimit(0,0); 
    y_joint->SetUpperLimit(0,0);
    z_joint->SetLowerLimit(0,0.2); 
    z_joint->SetUpperLimit(0,0.2);  
    yaw_joint->SetLowerLimit(0,0); 
    yaw_joint->SetUpperLimit(0,0); 
    pitch_joint->SetLowerLimit(0,0); 
    pitch_joint->SetUpperLimit(0,0); 
    static_joint_attached_ = true;
    // Reset simulation flag
    runSim_ = false;
}


void CassiePlugin::onUpdate()
{   
    // Get current time and calculate last update time
    auto currentTime = worldPtr_->SimTime();
    auto secondsSinceLastUpdate = (currentTime - lastUpdateTime_).Double();

    // Run at the specified period
    if (secondsSinceLastUpdate < updatePeriod_*0.95)
        return;

    // Zero the input if no packets have been received in a while
    if ((currentTime - lastPacketTime_).Double() > 0.01)
        cassieUserIn_ = {};

    // Get newest packet, or return -1 if no new packets are available
    addrLen_ = sizeof srcAddr_;
    ssize_t nbytes = get_newest_packet(sock_, recvBuf_, RECVLEN,
                                       (struct sockaddr *) &srcAddr_,
                                       &addrLen_);
    cassie_linux_data_t linux_data;

    // If a new packet was received, process and unpack it
    if (RECVLEN == nbytes) {
        // Process incoming header and write outgoing header
        process_packet_header(&headerInfo_, headerInPtr_, headerOutPtr_);

        // Unpack received data into cassie user input struct
        // unpack_cassie_user_in_t(dataInPtr_, &cassieUserIn_);
        unpack_cassie_linux_data_t(dataInPtr_, &linux_data);

        // Start running Cassie core after the first valid packet is received
        if (!runSim_) {
            runSim_ = true;
            firstPacketTime_ = currentTime;
        }

        // Record time
        lastPacketTime_ = currentTime;
    }
    
    if (runSim_) {
        // // Slowly lower and detach robot for easy initialization
        // const double LOWER_TIME = 1.0;
        // const double DETACH_TIME = 3.0;
        // if (static_joint_attached_) {
        //     if ((currentTime - firstPacketTime_).Double() > LOWER_TIME && (currentTime - firstPacketTime_).Double() < DETACH_TIME) {
        //         // Lower pelvis 1 seconds after receiving data
        //         lowerPelvis();
        //     } else if ((currentTime - firstPacketTime_).Double() > DETACH_TIME) {
        //         // Detatch pelvis 5 seconds after receiving data
        //         detachPelvis();
        //     }
        // }
        // Call low-level controller
        lowLevelController(&linux_data);

        // Run simulator and pack output struct into outgoing packet
        cassie_in_t cassieIn;
        cassie_out_t output = cassieOut_;
        cassie_core_sim_step(corePtr_, &cassieUserIn_, &output, &cassieIn);
        applyTorques(&cassieIn);
        updateCassieOut();
        lastUpdateTime_ += gazebo::common::Time(updatePeriod_);

        // Get simulated input from keyboard
        // LV, LH, RV, RH, S1, S2, LS, RS, SA, SB, SC, SD, SE, SF, SG, SH 
        output.pelvis.radio.channel[RadioButtons::LV] = keyboard_.LV();
        output.pelvis.radio.channel[RadioButtons::LH] = keyboard_.LH();
        output.pelvis.radio.channel[RadioButtons::RV] = keyboard_.RV();
        output.pelvis.radio.channel[RadioButtons::RH] = keyboard_.RH();
        output.pelvis.radio.channel[RadioButtons::S1] = keyboard_.S1();
        output.pelvis.radio.channel[RadioButtons::S2] = keyboard_.S2();
        output.pelvis.radio.channel[RadioButtons::LS] = keyboard_.LS();
        output.pelvis.radio.channel[RadioButtons::RS] = keyboard_.RS();
        output.pelvis.radio.channel[RadioButtons::SA] = keyboard_.SA();
        output.pelvis.radio.channel[RadioButtons::SB] = keyboard_.SB();
        output.pelvis.radio.channel[RadioButtons::SC] = keyboard_.SC();
        output.pelvis.radio.channel[RadioButtons::SD] = keyboard_.SD();
        output.pelvis.radio.channel[RadioButtons::SE] = keyboard_.SE();
        output.pelvis.radio.channel[RadioButtons::SF] = keyboard_.SF();
        output.pelvis.radio.channel[RadioButtons::SG] = keyboard_.SG();
        output.pelvis.radio.channel[RadioButtons::SH] = keyboard_.SH();
        // print out current radio value
        std::cout<<"LV="<<output.pelvis.radio.channel[RadioButtons::LV]<<"\n";
        std::cout<<"LH="<<output.pelvis.radio.channel[RadioButtons::LH]<<"\n";
        std::cout<<"RV="<<output.pelvis.radio.channel[RadioButtons::RV]<<"\n";
        std::cout<<"RH="<<output.pelvis.radio.channel[RadioButtons::RH]<<"\n";
        std::cout<<"S1="<<output.pelvis.radio.channel[RadioButtons::S1]<<"\n";
        std::cout<<"S2="<<output.pelvis.radio.channel[RadioButtons::S2]<<"\n";
        std::cout<<"LS="<<output.pelvis.radio.channel[RadioButtons::LS]<<"\n";
        std::cout<<"RS="<<output.pelvis.radio.channel[RadioButtons::RS]<<"\n";
        std::cout<<"SA="<<output.pelvis.radio.channel[RadioButtons::SA]<<"\n";
        std::cout<<"SB="<<output.pelvis.radio.channel[RadioButtons::SB]<<"\n";
        std::cout<<"SC="<<output.pelvis.radio.channel[RadioButtons::SC]<<"\n";
        std::cout<<"SD="<<output.pelvis.radio.channel[RadioButtons::SD]<<"\n";
        std::cout<<"SE="<<output.pelvis.radio.channel[RadioButtons::SE]<<"\n";
        std::cout<<"SF="<<output.pelvis.radio.channel[RadioButtons::SF]<<"\n";
        std::cout<<"SG="<<output.pelvis.radio.channel[RadioButtons::SG]<<"\n";
        std::cout<<"SH="<<output.pelvis.radio.channel[RadioButtons::SH]<<"\n";

        cassie_slrt_data_t slrt_data;   
        memset(&slrt_data, 0, sizeof (cassie_slrt_data_t));        
        slrt_data.outputs = output;
        slrt_data.t = lastUpdateTime_.Double();
        // pack_cassie_out_t(&output, dataOutPtr_);
        pack_cassie_slrt_data_t(&slrt_data, dataOutPtr_);

        // Send response
        send_packet(sock_, sendBuf_, SENDLEN,
                    (struct sockaddr *) &srcAddr_, addrLen_);

        slrt_data_prev_ = slrt_data;
    }
}


void CassiePlugin::updateCassieOut()
{
    // Motor encoders
    for (size_t i = 0; i < 10; ++i) {
        setMotorEncoder(driveOut_[i], motor_[i]->Position(0) + kMotorOffset_[i],
                        &motorFilter_[i], kMotorBits_[i]);
    }

    // Joint encoders
    for (size_t i = 0; i < 6; ++i) {
        setJointEncoder(jointOut_[i], joint_[i]->Position(0) + kJointOffset_[i],
                        &jointFilter_[i], kJointBits_[i]);
    }

    // IMU
    auto pose = pelvisPtr_->WorldPose();

    auto worldAccel = pelvisPtr_->WorldLinearAccel() - worldPtr_->Gravity();
    auto worldGyro = pelvisPtr_->WorldAngularVel();
    auto worldMag = ignition::math::Vector3d(0, 1, 0);

    auto rot = pose.Rot().Inverse();
    auto accel = rot.RotateVector(worldAccel);
    auto gyro = rot.RotateVector(worldGyro);
    auto mag = rot.RotateVector(worldMag);

    // Set computed orientation
    cassieOut_.pelvis.vectorNav.orientation[0] = pose.Rot().W();
    cassieOut_.pelvis.vectorNav.orientation[1] = pose.Rot().X();
    cassieOut_.pelvis.vectorNav.orientation[2] = pose.Rot().Y();
    cassieOut_.pelvis.vectorNav.orientation[3] = pose.Rot().Z();

    // Set accelerometer/gyro/magnetometer measurement
    for (size_t i = 0; i < 3; ++i) {
        cassieOut_.pelvis.vectorNav.linearAcceleration[i] = accel[i];
        cassieOut_.pelvis.vectorNav.angularVelocity[i] = gyro[i];
        cassieOut_.pelvis.vectorNav.magneticField[i] = mag[i];
    }
}


void CassiePlugin::applyTorques(const cassie_in_t *cassieIn)
{
    // STO
    bool sto = this->cassieOut_.pelvis.radio.channel[8] < 1;

    // Get torque commands
    const double torque[] = {
        cassieIn->leftLeg.hipRollDrive.torque,
        cassieIn->leftLeg.hipYawDrive.torque,
        cassieIn->leftLeg.hipPitchDrive.torque,
        cassieIn->leftLeg.kneeDrive.torque,
        cassieIn->leftLeg.footDrive.torque,
        cassieIn->rightLeg.hipRollDrive.torque,
        cassieIn->rightLeg.hipYawDrive.torque,
        cassieIn->rightLeg.hipPitchDrive.torque,
        cassieIn->rightLeg.kneeDrive.torque,
        cassieIn->rightLeg.footDrive.torque
    };

    // Set and output limited torque commands
    for (size_t i = 0; i < 10; ++i) {
        driveOut_[i]->torque = setMotor(motor_[i], torque[i], sto,
                                        kGearRatio_[i], kMaxTorque_[i],
                                        kMaxSpeed_[i]);
    }
}


void CassiePlugin::detachPelvis() {
    // Set large limits to effectively detach
    auto model = this->worldPtr_->ModelByName("cassie");
    gazebo::physics::JointPtr x_joint = model->GetJoint("x");
    gazebo::physics::JointPtr y_joint = model->GetJoint("y");
    gazebo::physics::JointPtr z_joint = model->GetJoint("z");
    gazebo::physics::JointPtr yaw_joint = model->GetJoint("yaw");
    gazebo::physics::JointPtr pitch_joint = model->GetJoint("pitch");
    x_joint->SetLowerLimit(0,-1000); 
    x_joint->SetUpperLimit(0,1000); 
    y_joint->SetLowerLimit(0,-1000); 
    y_joint->SetUpperLimit(0,1000);
    z_joint->SetLowerLimit(0,-1000); 
    z_joint->SetUpperLimit(0,1000);  
    yaw_joint->SetLowerLimit(0,-1000); 
    yaw_joint->SetUpperLimit(0,1000); 
    pitch_joint->SetLowerLimit(0,-1000); 
    pitch_joint->SetUpperLimit(0,1000); 
    static_joint_attached_ = false;
}


void CassiePlugin::lowerPelvis() {   
    // Lower by small amount each timestep
    gazebo::physics::JointPtr joint = this->worldPtr_->ModelByName("cassie")->GetJoint("z");
    joint->SetLowerLimit(0,joint->LowerLimit()-0.0005); 
}


void CassiePlugin::lowLevelController(cassie_linux_data_t* linux_data){
    // Store default user inputs
    cassieUserIn_ = linux_data->userInputs;

    // Extract motor positions and velocities
    double jointPositions[10] = {slrt_data_prev_.outputs.leftLeg.hipRollDrive.position,
        slrt_data_prev_.outputs.leftLeg.hipYawDrive.position,
        slrt_data_prev_.outputs.leftLeg.hipPitchDrive.position,
        slrt_data_prev_.outputs.leftLeg.kneeDrive.position,
        slrt_data_prev_.outputs.leftLeg.footDrive.position,
        slrt_data_prev_.outputs.rightLeg.hipRollDrive.position,
        slrt_data_prev_.outputs.rightLeg.hipYawDrive.position,
        slrt_data_prev_.outputs.rightLeg.hipPitchDrive.position,
        slrt_data_prev_.outputs.rightLeg.kneeDrive.position,
        slrt_data_prev_.outputs.rightLeg.footDrive.position};
    double jointVelocities[10] = {slrt_data_prev_.outputs.leftLeg.hipRollDrive.velocity,
        slrt_data_prev_.outputs.leftLeg.hipYawDrive.velocity,
        slrt_data_prev_.outputs.leftLeg.hipPitchDrive.velocity,
        slrt_data_prev_.outputs.leftLeg.kneeDrive.velocity,
        slrt_data_prev_.outputs.leftLeg.footDrive.velocity,
        slrt_data_prev_.outputs.rightLeg.hipRollDrive.velocity,
        slrt_data_prev_.outputs.rightLeg.hipYawDrive.velocity,
        slrt_data_prev_.outputs.rightLeg.hipPitchDrive.velocity,
        slrt_data_prev_.outputs.rightLeg.kneeDrive.velocity,
        slrt_data_prev_.outputs.rightLeg.footDrive.velocity};

    // Store state in eigen form
    Eigen::Matrix<double,20,1> q, dq;

    // Set robot's pose using estimated state
    q(0) = linux_data->state.q[0];
    q(1) = linux_data->state.q[1];
    q(2) = linux_data->state.q[2];
    q(3) = linux_data->state.q[3];
    q(4) = linux_data->state.q[4];
    q(5) = linux_data->state.q[5];
    // Set encoders left leg (using potentially higher frequency encoder feedback)
    q(6) = slrt_data_prev_.outputs.leftLeg.hipRollDrive.position; 
    q(7) = slrt_data_prev_.outputs.leftLeg.hipYawDrive.position;
    q(8) = slrt_data_prev_.outputs.leftLeg.hipPitchDrive.position;
    q(9) = slrt_data_prev_.outputs.leftLeg.kneeDrive.position;
    q(10) = slrt_data_prev_.outputs.leftLeg.shinJoint.position;
    q(11) = slrt_data_prev_.outputs.leftLeg.tarsusJoint.position;
    q(12) = slrt_data_prev_.outputs.leftLeg.footDrive.position;
    // Set encoders right leg (using potentially higher frequency encoder feedback)
    q(13) = slrt_data_prev_.outputs.rightLeg.hipRollDrive.position;
    q(14) = slrt_data_prev_.outputs.rightLeg.hipYawDrive.position;
    q(15) = slrt_data_prev_.outputs.rightLeg.hipPitchDrive.position;
    q(16) = slrt_data_prev_.outputs.rightLeg.kneeDrive.position;
    q(17) = slrt_data_prev_.outputs.rightLeg.shinJoint.position;
    q(18) = slrt_data_prev_.outputs.rightLeg.tarsusJoint.position;
    q(19) = slrt_data_prev_.outputs.rightLeg.footDrive.position;

    // Set robot's pose velocity using estimated state
    dq(0) = linux_data->state.dq[0];
    dq(1) = linux_data->state.dq[1];
    dq(2) = linux_data->state.dq[2];
    dq(3) = linux_data->state.dq[3];
    dq(4) = linux_data->state.dq[4];
    dq(5) = linux_data->state.dq[5];
    // Set encoder velocities left leg (using potentially higher frequency encoder feedback)
    dq(6) = slrt_data_prev_.outputs.leftLeg.hipRollDrive.velocity;
    dq(7) = slrt_data_prev_.outputs.leftLeg.hipYawDrive.velocity;
    dq(8) = slrt_data_prev_.outputs.leftLeg.hipPitchDrive.velocity;
    dq(9) = slrt_data_prev_.outputs.leftLeg.kneeDrive.velocity;
    dq(10) = slrt_data_prev_.outputs.leftLeg.shinJoint.velocity;
    dq(11) = slrt_data_prev_.outputs.leftLeg.tarsusJoint.velocity;
    dq(12) = slrt_data_prev_.outputs.leftLeg.footDrive.velocity;
    // Set encoder velocities right leg (using potentially higher frequency encoder feedback)
    dq(13) = slrt_data_prev_.outputs.rightLeg.hipRollDrive.velocity;
    dq(14) = slrt_data_prev_.outputs.rightLeg.hipYawDrive.velocity;
    dq(15) = slrt_data_prev_.outputs.rightLeg.hipPitchDrive.velocity;
    dq(16) = slrt_data_prev_.outputs.rightLeg.kneeDrive.velocity;
    dq(17) = slrt_data_prev_.outputs.rightLeg.shinJoint.velocity;
    dq(18) = slrt_data_prev_.outputs.rightLeg.tarsusJoint.velocity;
    dq(19) = slrt_data_prev_.outputs.rightLeg.footDrive.velocity;

    // Choose control type
    switch (linux_data->control.type) {
        case ControlType::Torque: {
            for (unsigned int i = 0; i < 10; i++) {
                cassieUserIn_.torque[i] = linux_data->userInputs.torque[i];
            }
            break;
        }
        case ControlType::Position: {
            for (unsigned int i = 0; i < 10; i++) {
                cassieUserIn_.torque[i] = Kp_PD_[i] * (linux_data->control.motorPositionsDesired[i] - jointPositions[i]) +
                                          Kd_PD_[i] * (linux_data->control.motorVelocitiesDesired[i] - jointVelocities[i]);
            }
            break;
        }
        case ControlType::Position_GC: {
            Eigen::Matrix<double,20,1> G = Ge_cassie(q);
            Eigen::Matrix<double,10,20> Bpinv = Eigen::Matrix<double,10,20>::Zero();
            Bpinv(0,6) = 1;
            Bpinv(1,7) = 1;
            Bpinv(2,8) = 1;
            Bpinv(3,9) = 1;
            Bpinv(4,12) = 1;
            Bpinv(5,13) = 1;
            Bpinv(6,14) = 1;
            Bpinv(7,15) = 1;
            Bpinv(8,16) = 1;
            Bpinv(9,19) = 1;
            Eigen::Matrix<double,10,1> uff = Bpinv*G;
            // std::cout << "uff:\n" << uff << "\n\n";

            std::cout << "ControlType::Position_GC not working yet!!!!!\n";
            std::cout << "ufb:\n";
            for (unsigned int i = 0; i < 10; i++) {
                cassieUserIn_.torque[i] = 0*uff(i) + 
                                          0*Kp_PD_[i] * (linux_data->control.motorPositionsDesired[i] - jointPositions[i]) +
                                          0*Kd_PD_[i] * (linux_data->control.motorVelocitiesDesired[i] - jointVelocities[i]);
                std::cout << cassieUserIn_.torque[i] << "\n";
            }
            std::cout << "\n\n";
            break;
        }
        default: {
            // Error
        }
    }
    
    // Call safety controller (modifies torques)
    // safetyController();
}


void CassiePlugin::safetyController(){
    // Torque smoothing
    bool update_smoothing_flag = false;
    for (unsigned int i = 0; i < 10; i++) {
        if (std::abs(cassieUserIn_.torque[i] - torques_unmodified_prev_[i]) > torque_discontinuity_threshold_) {
            update_smoothing_flag = true;
            break;
        }
    }
    if (update_smoothing_flag) { // Reset sigmoid parameters
        u_cont_t0_ = slrt_data_prev_.t;
        for (unsigned int i = 0; i < 10; i++){
            u_cont_alpha_[i] = cassieUserIn_.torque[i] - torques_unmodified_prev_[i];
        }
    }
    for (unsigned int i = 0; i < 10; i++){
        torques_unmodified_prev_[i] = cassieUserIn_.torque[i];
    }
    for (unsigned int i = 0; i < 10; i++){
        cassieUserIn_.torque[i] += -2*u_cont_alpha_[i]*(1 - 1/(1 + std::exp(-u_cont_beta_*(slrt_data_prev_.t - u_cont_t0_))));
    }
}



// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(CassiePlugin)
