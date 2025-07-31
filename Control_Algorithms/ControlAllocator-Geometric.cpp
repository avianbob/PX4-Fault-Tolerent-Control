/**************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************/

/**
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */
//#include <iostream>
//#include <eigen3/Eigen/Dense>
#include <cmath>
#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>


//declaring things failure_flag.h in the code
#include <uORB/topics/failure_flag.h> 
//control_allocator // printing
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <matrix/math.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

using namespace matrix;
using namespace time_literals;
//declaration of variables

//for failure detection
bool fail_change;
int failM_index;
bool check=false;
bool geo;

//for control algo
float roll, pitch, yaw;
float roll_rate, pitch_rate, yaw_rate;
float altitude,position_x,position_y;
float velocity_x,velocity_y,velocity_z;
float desired_roll, desired_pitch, desired_yaw, thrust_setpoint;
double latitude, longitude;
Vector3f torque_output;
float thrust_output;
Vector4f motor_outputs;
float pi = 3.14;
//control parammeters for geometric controller-1
static float roll_sp, pitch_sp, yaw_sp;
static bool calibrate;
constexpr float KP_ATTITUDE = 5.0f;    // Proportional gain for attitude
constexpr float KD_ATTITUDE = 0.5f;    // Derivative gain for angular rates
constexpr float MAX_THRUST = 1.0f;     // Max thrust for each motor
constexpr float MIN_THRUST = 0.0f; 
constexpr float L = 0.2f;              // Lever arm length (meters)
constexpr float D = 0.1f; 
/********************************************************************************************** */
/*The below code is defined for geometric controller along with the MRAC(model response adaptive control)
this code takes the input after detection of failsafe from the failure_flag.msg after suscribing to it.
The code takes input as roll_rate,pitch_rate and yaw_rate from the vehical angular velocity .
local position such as altitude from vehicle_local_position uORB topics 
then it performs calculation according to the mentioned control equations in the outer , inner and allocation loops and then publishes motor rpm values after normalisation to the actuator_optputs topics*/ 
/********************************************************************************************** */
/////////////////////////////************************************************************//////////////////////////////////////////// */
//main control parameters for geometric control

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <matrix/matrix/math.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_outputs.h>
#include <drivers/drv_hrt.h>

using namespace matrix;

// Drone parameters
const double mass = 0.9;
double k_p = 7 * mass;
double k_v = 7 * mass;
double k_R = 6.3;
double k_Omega = 0.55;
const float gravity = 9.81;

Matrix3f J; // Global declaration
//inertia matrix
void initializeMatrixJ() {
    J(0, 0) = 0.0024657f; J(0, 1) = 0.0f;   J(0, 2) = 0.0f;
    J(1, 0) = 0.0f;   J(1, 1) = 0.0027657f; J(1, 2) = 0.0f;
    J(2, 0) = 0.0f;   J(2, 1) = 0.0f;   J(2, 2) = 0.0042354f;
}


// Rotor constants
const double thrust_coeff = 1024.0 / 3.0;
const double distance = 0.1125;

// Adaptive MRAC rates
double alpha_p = 0.000001;
double alpha_v = 0.000001;
double alpha_R = 0.000001;
double alpha_Omega = 0.000001;

// Desired states
Vector3f position_desired{1.0f, 1.0f, -5.0f};
Vector3f velocity_desired{0.0f, 0.0f, 0.0f};
Vector3f Omega_desired{0.0f, 0.0f, 0.0f};

Matrix3f Rotational_matrix_desired;
void initialize_rot_matrix(){
	Rotational_matrix_desired(0,0) =1.0f; Rotational_matrix_desired(0,1) =0.0f; Rotational_matrix_desired(0,2) =0.0f;
	Rotational_matrix_desired(1,0) =0.0f; Rotational_matrix_desired(1,1) =1.0f; Rotational_matrix_desired(1,2) =0.0f;
	Rotational_matrix_desired(2,0) =0.0f; Rotational_matrix_desired(2,1) =0.0f; Rotational_matrix_desired(2,2) =1.0f;
} 

// Clamp function
double clamp(double value, double min_val, double max_val) {
    return fmax(min_val, fmin(value, max_val));
}

// Function to adaptively tune controller gains using MRAC
void adaptGains(const Vector3f &error_position, const Vector3f &error_velocity, 
                const Vector3f &error_Rotational_matrix, const Vector3f &error_Omega) {
    k_p += mass * alpha_p * (double)error_position.dot(error_position);
    k_v += mass * alpha_v * (double)error_velocity.dot(error_velocity);
    k_R += alpha_R * (double)error_Rotational_matrix.dot(error_Rotational_matrix);
    k_Omega += alpha_Omega * (double)error_Omega.dot(error_Omega);

    k_p = clamp(k_p, 0.1, 500.0);
    k_v = clamp(k_v, 0.1, 500.0);
    k_R = clamp(k_R, 0.1, 500.0);
    k_Omega = clamp(k_Omega, 0.1, 500.0);
}

// Global matrix B for thrust coefficients and force distribution
matrix::Matrix<float, 3, 4> B;
void initialiseB() {
    B(0, 0) = thrust_coeff; B(0, 1) = thrust_coeff; B(0, 2) = thrust_coeff; B(0, 3) = thrust_coeff;
    B(1, 0) = 0.0f;         B(1, 1) = 0.0f;         B(1, 2) = distance * thrust_coeff; B(1, 3) = -distance * thrust_coeff;
    B(2, 0) = -distance * thrust_coeff; B(2, 1) = distance * thrust_coeff; B(2, 2) = 0.0f; B(2, 3) = 0.0f;
}

// Function to calculate rotor speeds for fault-tolerant control
matrix::Vector<float, 4> calculateRotorSpeeds(int failed_rotor_index, const matrix::Vector<float, 3> &force_total) {
    initialiseB();  // Initialize matrix B

    // Check for a valid rotor index and set the corresponding column to zero
    if (failed_rotor_index >= 0 && failed_rotor_index < 4) {
       // Set the entire column to zero
		for (int i = 0; i < 3; ++i) {
    		B(i, failed_rotor_index) = 0.0f;
		}

    }
//orthogonal matri decomposition
    // Step 1: Gram-Schmidt orthogonalization
    matrix::Matrix<float, 3, 4> Q;  // Orthogonal matrix
    matrix::Matrix<float, 4, 4> R;  // Upper triangular matrix

    // Gram-Schmidt Process
    for (int j = 0; j < 4; ++j) {
        // Step 1: Start by assigning the j-th column of B to the j-th column of Q
        Q(0, j) = B(0, j);
        Q(1, j) = B(1, j);
        Q(2, j) = B(2, j);

        // Step 2: Orthogonalize the j-th column against all previous columns in Q
        for (int i = 0; i < j; ++i) {
            float dot_product = Q(0, i) * Q(0, j) + Q(1, i) * Q(1, j) + Q(2, i) * Q(2, j);
            for (int k = 0; k < 3; ++k) {
                Q(k, j) -= dot_product * Q(k, i);
            }
        }

        // Step 3: Normalize the j-th column of Q
        float norm = std::sqrt(Q(0, j) * Q(0, j) + Q(1, j) * Q(1, j) + Q(2, j) * Q(2, j));
        for (int k = 0; k < 3; ++k) {
            Q(k, j) /= norm;
        }
    }

    // Step 2: Calculate the upper triangular matrix R (since Q is orthogonal, B = Q * R)
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j <= i; ++j) {
            R(i, j) = Q(0, i) * B(0, j) + Q(1, i) * B(1, j) + Q(2, i) * B(2, j);
        }
    }

    // Step 3: Solve for omega_squared (since B = Q * R, omega_squared = R^(-1) * Q^T * total_force)
    matrix::Vector<float, 4> temp = Q.transpose() * force_total;
	matrix::Vector<float, 4> omega_squared;
    // Solve R * omega_squared = temp (since R is upper triangular, we can solve it using backward substitution)
    omega_squared(3) = temp(3) / R(3, 3);
    omega_squared(2) = (temp(2) - R(2, 3) * omega_squared(3)) / R(2, 2);
    omega_squared(1) = (temp(1) - R(1, 2) * omega_squared(2) - R(1, 3) * omega_squared(3)) / R(1, 1);
    omega_squared(0) = (temp(0) - R(0, 1) * omega_squared(1) - R(0, 2) * omega_squared(2) - R(0, 3) * omega_squared(3)) / R(0, 0);

    // Output the result
    // Ensure that omega_squared values are non-negative
    for (int i = 0; i < 4; ++i) {  // Since omega_squared has 4 elements
    omega_squared(i) = fmax(0.0f, omega_squared(i));
	}

    return omega_squared;
}



// Main controller logic
void geometric_controller_main_logic() {
	initializeMatrixJ();
	initialize_rot_matrix();
    static int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    static int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    static int angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    static orb_advert_t actuator_outputs_pub = nullptr;

    struct vehicle_local_position_s local_pos;
    struct vehicle_attitude_s attitude;
    struct vehicle_angular_velocity_s angular_velocity;

    if (orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos) != PX4_OK ||
        orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude) != PX4_OK ||
        orb_copy(ORB_ID(vehicle_angular_velocity), angular_velocity_sub, &angular_velocity) != PX4_OK) {
        PX4_ERR("Failed to copy vehicle state topics.");
        return;
    }

    // Current states
    Vector3f position(local_pos.x, local_pos.y, local_pos.z);
    Vector3f velocity(local_pos.vx, local_pos.vy, local_pos.vz);
    Quatf q(attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]);
    Vector3f Omega(angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]);
	matrix::Matrix3f Rotational_matrix_current;

	// Extract the components of the quaternion
	float w = q(0);
	float x = q(1);
	float y = q(2);
	float z = q(3);

	// Manually calculate the DCM (rotation matrix) from the quaternion
	Rotational_matrix_current(0, 0) = 1 - 2 * (y * y + z * z);
	Rotational_matrix_current(0, 1) = 2 * (x * y - z * w);
	Rotational_matrix_current(0, 2) = 2 * (x * z + y * w);

	Rotational_matrix_current(1, 0) = 2 * (x * y + z * w);
	Rotational_matrix_current(1, 1) = 1 - 2 * (x * x + z * z);
	Rotational_matrix_current(1, 2) = 2 * (y * z - x * w);

	Rotational_matrix_current(2, 0) = 2 * (x * z - y * w);
	Rotational_matrix_current(2, 1) = 2 * (y * z + x * w);
	Rotational_matrix_current(2, 2) = 1 - 2 * (x * x + y * y);
	
    // Compute errors
    Vector3f error_position = position - position_desired;
    Vector3f error_velocity = velocity - velocity_desired;
    Matrix3f e_R_mat = 0.5f * (Rotational_matrix_desired.transpose() * Rotational_matrix_current -
                              Rotational_matrix_current.transpose() * Rotational_matrix_desired);
    Vector3f error_Rotational_matrix(e_R_mat(2, 1), e_R_mat(0, 2), e_R_mat(1, 0));
    Vector3f error_Omega = Omega - Rotational_matrix_current.transpose() * Rotational_matrix_desired * Omega_desired;

    // Adapt gains
    adaptGains(error_position, error_velocity, error_Rotational_matrix, error_Omega);

    // Compute thrust and torque commands
    Vector3f thrust = (float)mass * (gravity * Vector3f(0, 0, 1) - (float)k_p * error_position - (float)k_v * error_velocity);
    Vector3f torque = (float)-k_R * error_Rotational_matrix - (float)k_Omega * error_Omega;

    Vector3f force_total(thrust.norm(), torque(0), torque(1));

    // Fault-tolerant control for rotor failure (e.g., rotor 3 fails)
    int failed_rotor_index = 3;
    Vector4f rotor_speeds = calculateRotorSpeeds(failed_rotor_index, force_total);

    // Publish actuator outputs
    struct actuator_outputs_s actuator_outputs = {};
    actuator_outputs.output[0] = rotor_speeds(0);
    actuator_outputs.output[1] = rotor_speeds(1);
    actuator_outputs.output[2] = rotor_speeds(2);
    actuator_outputs.output[3] = rotor_speeds(3);
    actuator_outputs.timestamp = hrt_absolute_time();

    if (actuator_outputs_pub == nullptr) {
        actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);
    } else {
        orb_publish(ORB_ID(actuator_outputs), actuator_outputs_pub, &actuator_outputs);
    }
}

/****************///////////////////////////////////////////////////////////*************************************/////////////////////////////////////// */ */
// Add a member variable for the subscription
uORB::Subscription _failure_flag_sub{ORB_ID(failure_flag)}; //suscription to custom made failure_flag.
/**********************************///////////////////////////////*************** */ */
void fetchFlightData() {
    // Subscriptions
    static uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    static uORB::Subscription vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    static uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    static uORB::Subscription vehicle_gps_position_sub{ORB_ID(sensor_gps)};

    // Data structures
    vehicle_attitude_s attitude;
    vehicle_angular_velocity_s angular_velocity;
    vehicle_local_position_s local_position;
    sensor_gps_s gps_data;

    // Roll, Pitch, Yaw (from vehicle_attitude)
    if (vehicle_attitude_sub.update(&attitude)) {
        matrix::Quatf q(attitude.q[0], attitude.q[1],attitude.q[2], attitude.q[3]);
        matrix::Eulerf euler(q);
		roll = math::degrees(euler.phi());
        pitch = math::degrees(euler.theta());
        yaw = math::degrees(euler.psi());
      // Yaw in radians
    }

    // Angular Rates (from vehicle_angular_velocity)
    if (vehicle_angular_velocity_sub.update(&angular_velocity)) {
        roll_rate = angular_velocity.xyz[0]*180 / pi;  // Roll rate in rad/s
        pitch_rate = angular_velocity.xyz[1]*180 / pi; // Pitch rate in rad/s
        yaw_rate = angular_velocity.xyz[2]*180 / pi;   // Yaw rate in rad/s
    }

    // Altitude (from vehicle_local_position)
    if (vehicle_local_position_sub.update(&local_position)) {
        altitude = local_position.z;  // Altitude in meters
		position_x=local_position.x;
		position_y=local_position.y;

		velocity_x= local_position.vx;
		velocity_y= local_position.vy;
		velocity_z= local_position.vz;
    }

    // Latitude, Longitude (from vehicle_gps_position)
    if (vehicle_gps_position_sub.update(&gps_data)) {
        latitude = gps_data.latitude_deg;    // Latitude in degrees
        longitude = gps_data.longitude_deg; //longitude in degrees
    }
	// Calibration logic
                if (!calibrate) {
                    roll_sp = roll;    // Set roll setpoint
                    pitch_sp = pitch;  // Set pitch setpoint
                    yaw_sp = yaw;      // Set yaw setpoint
                    calibrate = true;
                }
}
void update_attitude_setpoint() {
    uORB::Subscription vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    vehicle_attitude_setpoint_s attitude_setpoint;

    if (vehicle_attitude_setpoint_sub.update(&attitude_setpoint)) {
        // Convert quaternion setpoint to Euler angles (desired roll, pitch, yaw)
        matrix::Quatf q_setpoint(attitude_setpoint.q_d[0], attitude_setpoint.q_d[1], 
                                 attitude_setpoint.q_d[2], attitude_setpoint.q_d[3]);
        matrix::Eulerf euler_setpoint(q_setpoint);  // Convert quaternion to Euler angles

        // Extract desired roll, pitch, yaw in radians from the quaternion
        desired_roll = euler_setpoint.phi();    // Desired roll (radians)
        desired_pitch = euler_setpoint.theta(); // Desired pitch (radians)
        desired_yaw = euler_setpoint.psi();     // Desired yaw (radians)

        // Extract thrust setpoint (z-component in body frame)
        thrust_setpoint = attitude_setpoint.thrust_body[2]; // Thrust along z-axis
    } else {
        PX4_ERR("Failed to update vehicle attitude setpoint");
    }
}

void publish_actuator_commands() {
    // Prepare actuator_controls_0 message
    actuator_motors_s actuator_motors = {};
    actuator_motors.timestamp = hrt_absolute_time();

	//limit outputs
	for(int i=0;i<4;i++){
		if(motor_outputs(i)>2000){
			motor_outputs(i)=2000;
		}
		else if(motor_outputs(i)<0){
			motor_outputs(i)=0;
		}
	}
	//normalise the output
	for(int i=0;i<4;i++){
		motor_outputs(i)=motor_outputs(i)/(2000);
	}

    // Assign motor outputs to actuator controls
    actuator_motors.control[0] = motor_outputs(0); // Motor 1
    actuator_motors.control[1] = motor_outputs(1); // Motor 2
    actuator_motors.control[2] = motor_outputs(2); // Motor 3
    actuator_motors.control[3] = motor_outputs(3);             // Reserve for other motors if needed

    // Publish the message
    uORB::Publication<actuator_motors_s> actuator_controls_pub{ORB_ID(actuator_motors)};
    actuator_controls_pub.publish(actuator_motors);
	// PX4_INFO("Torque: %.3f, %.3f, %.3f | Thrust: %.3f", (double)torque_output(0), (double)torque_output(1), (double)torque_output(2), (double)thrust_output);
    // PX4_INFO("Motor Outputs: %.3f, %.3f, %.3f", (double)motor_outputs(0), (double)motor_outputs(1), (double)motor_outputs(2));

}

void processAttitudeAndStatus() {
    // Subscriptions to vehicle_attitude and vehicle_status
    static uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    static uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
    vehicle_attitude_s attitude;
    vehicle_status_s status;

    // Update vehicle_status
    if (vehicle_status_sub.update(&status)) {
        // Debug: Print the current arming and navigation states
        PX4_DEBUG("Arming State: %d, Navigation State: %d", status.arming_state, status.nav_state);
		fetchFlightData();
        // Check if the drone is armed and not in a landed or disarmed state
        if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
            status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {

            // Update vehicle_attitude
            if (vehicle_attitude_sub.update(&attitude)) {
                // Debug information
               // PX4_INFO("Pitch: %.3f, Roll: %.3f, Yaw: %.3f", (double)pitch, (double)roll, (double)yaw);
    			//PX4_INFO("Roll Rate: %.3f, Pitch Rate: %.3f, Yaw Rate: %.3f", (double)roll_rate, (double)pitch_rate, (double)yaw_rate);
    			//PX4_INFO("Altitude: %.3f, Latitude: %.7f, Longitude: %.7f", (double)altitude, (double)latitude, (double)longitude);
				//PX4_INFO("Desired Roll: %.2f, Pitch: %.2f, Yaw: %.2f", (double)desired_roll, (double)desired_pitch, (double)desired_yaw);
				//PX4_INFO("Thrust Setpoint: %.2f", (double)thrust_setpoint);

				//PX4_INFO("Position - position_x: %f, position_y: %f", (double)position_x, (double)position_y);
        		//PX4_INFO("Velocity - velocity_x: %f, velocity_y: %f, velocity_z: %f", (double)velocity_x, (double)velocity_y, (double)velocity_z);
				//PX4_INFO("thrust0:%f,thrust1%f,thrust2%f",(double)thrust[0],(double)thrust[1],(double)thrust[2]);
				//PX4_INFO("force_total[0]:%f,force_total[1]:%f,force_total[2]:%f",(double)force_total[0],(double)force_total[1],(double)force_total[2]);
				//PX4_INFO("torque0:%f,torque1:%f,torque2:%f",(double)torque[0],(double)torque[1],(double)torque[2]);
				//PX4_INFO("motor(0): %f,motor(1): %f,motor(2): %f,motor(3): %f",(double)motor_outputs(0),(double)motor_outputs(1),(double)motor_outputs(2),(double)motor_outputs(3));
				//PX4_INFO("k_p%f,k_v%f,k_R%f,k_Omega%f",k_p,k_v,k_R,k_Omega);
            }
			 
			else {
                PX4_WARN("No data available from vehicle_attitude topic!");
            }
        } else {
            PX4_DEBUG("Drone is not flying. Current nav_state: %d", status.nav_state);
        }
    }
}

/***************************************************************///////////////////////////////////////********************** */ */
ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		matrix::Vector<float, NUM_ACTUATORS> actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN:
			tmp = new ActuatorEffectivenessRoverAckermann();
			break;

		case EffectivenessSource::ROVER_DIFFERENTIAL:
			// rover_differential_control does allocation and publishes directly to actuator_motors topic
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated() && !_armed) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

/******Commenting any code that could trigger failsafe or create problem in simulating motor failure******/
		// if (_handled_motor_failure_bitmask == 0) {
		// 	// We don't update the geometry after an actuator failure, as it could lead to unexpected results
		// 	// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
		// 	updateParams();
		// 	parameters_updated();
		// }
/**************************************************************************** */
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 0.005f) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}

	if (do_update) {
		_last_run = now;



		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			_control_allocation[i]->allocate();
			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}
	}

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
	processAttitudeAndStatus();
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}
/******Commenting any code that could trigger failsafe or create problem in simulating motor failure******/
		// Handle failed actuators
		// if (_handled_motor_failure_bitmask) {
		// 	actuator_idx = 0;
		// 	memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

		// 	for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		// 		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

		// 		if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
		// 			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

		// 			for (int i = 0; i < NUM_AXES; i++) {
		// 				matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
		// 			}
		// 		}

		// 		++actuator_idx_matrix[selected_matrix];
		// 		++actuator_idx;
		// 	}
		// }
/********************************************************************************************************************* */
		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}
/***************************************////////////////////////************************ */ */
/*The below code is for reciving failsafe messages using the failure_flag method.
 It sends the failure flag, motor index of the failed motor and the no. of failed motors.
 the motor index is stored in `failM_index and the flaf fail_change is triggered.
 further changes are made in publish_actuator_controls. 
 */
void ControlAllocator::updateFailureStatus()
{
    failure_flag_s failure_msg;

    // Check if there's a new message
    if (_failure_flag_sub.update(&failure_msg)) {
        // Handle the received message
        if (failure_msg.failure_detected) {
            PX4_WARN("Failure detected! Failed motor index: %d, Type: %d",
                     failure_msg.failed_motor_index, failure_msg.failure_type);

            // Perform specific actions based on the failure
			fail_change=true;
			failM_index=failure_msg.failed_motor_index;
        }
    }
}
/**************************////////////////////////////////********************************* */ */

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

/******Commenting any code that could trigger failsafe or create problem in simulating motor failure******/
	// actuator_servos_s actuator_servos;
	// actuator_servos.timestamp = actuator_motors.timestamp;
	// actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

/******Commenting any code that could trigger failsafe or create problem in simulating motor failure******/
	//uint32_t stopped_motors = _actuator_effectiveness->getStoppedMotors() | _handled_motor_failure_bitmask;
	
	
/*********************/	
	/*here we defined the method for injection of failure in a single motor based on achieveing a particular altiitude.
	the constant TAKEOFF_ALTITUDE shows the default take of altitude at which we are injecting failure.*/
	float TAKEOFF_ALTITUDE = 10.0f; // Define your threshold altitude for failure
    static bool disable_motor_0 = false; //declaration to disable the given motor.

    	
	
   	/*the below funtion is for triggering the failure at the above methioned altitude.*/
        	if (!disable_motor_0 && altitude < -TAKEOFF_ALTITUDE) {
                disable_motor_0 = true; // Trigger motor failure once

            
        	}
		updateFailureStatus();
        
    	
    	
/*********************/	 	
    	
    	
	// motors
	int motor_idx;

	for (motor_idx = 0; motor_idx < _num_actuators[0] && motor_idx < actuator_motors_s::NUM_CONTROLS; motor_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[motor_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
/***********************************************************************************************************************************/			
/*decoment the code and comment the above for directly changing the control alog for tunning or flying on geometric with failure at altitue of 10m */
		// /*this method mentions the way to disable a particular motor to simulate single/multiple motor failures.*/
		// if (disable_motor_0 && motor_idx == 0) {
		// 			static bool gradual=true;
		// 			float speed_reduced=0.8f;
		// 			if(!fail_change && gradual){
    	//         	actuator_motors.control[motor_idx+3] = speed_reduced;
		// 			speed_reduced=speed_reduced-0.1f;
    	//         //  actuator_motors.control[motor_idx+3] = 0.0f;
		// 		}
		
		// } 
		// else {
		//    actuator_motors.control[motor_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		// }
		// /*the below code mentions change of control to geometric controller based on the value of the altitude for tunning/ testing 
		// the geometric contreller along with in case of a single motor failure.*/
		// /*the below method calls the motor_idx for the particular failed method
		//  and the following code uses geometric controller for gaining staiblity with three motors.*/
		// if(fail_change && failM_index==motor_idx){
    	// 	fetchFlightData();
		// 	update_attitude_setpoint();
    	// 	// Step 2: Compute geometric control
		// 	geometric_controller_main_logic();

   		// 	 // Step 3: Publish actuator commands
    	// 	//compute_motor_outputs();
		// 	publish_actuator_commands();
		// 	if(!check){
		// 		PX4_INFO("CHANGED CONTROL");
		// 		check=true;
		// 	}
		// 	/*the original functions here are commented off.*/
		// 	// actuator_motors.control[motor_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		// 	// actuator_motors.control[motor_idx+1] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		// 	// actuator_motors.control[motor_idx+2] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		// 	// actuator_motors.control[motor_idx+3] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		// 	//PX4_INFO("trying to regain control");
		// }
        	
		// // if (stopped_motors & (1u << motor_idx)) {
		// // 	actuator_motors.control[motor_idx] = NAN;
		// // }
/******************************************************************************************************************************* */
/******************************************************************************************************************************* */
		if (disable_motor_0 && motor_idx == 0) {
    	        //  actuator_motors.control[motor_idx+3] = 0.0f;// for directly giving value zero.
				}
		else {
    		actuator_motors.control[motor_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		}
		
		/*the below code mentions change of control to geometric controller based on the value of the altitude for tunning/ testing 
		the geometric contreller along with in case of a single motor failure.*/
		if(altitude<-5.0f){
			geo=true;
			failM_index=3;
		}
		/*the below method calls the motor_idx for the particular failed method
		 and the following code uses geometric controller for gaining staiblity with three motors.*/
		if(geo){
    		fetchFlightData();
			update_attitude_setpoint();
    		// Step 2: Compute geometric control
			geometric_controller_main_logic();

   			 // Step 3: Publish actuator commands
			publish_actuator_commands();
			if(!check){
				PX4_INFO("CHANGED CONTROL");
				check=true;
			}
			/*the original functions here are commented off.*/
			// actuator_motors.control[motor_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			// actuator_motors.control[motor_idx+1] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			// actuator_motors.control[motor_idx+2] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			// actuator_motors.control[motor_idx+3] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			//PX4_INFO("trying to regain control");
		}
/******************************************************************************************************************************* */
		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
		
/*************************************************************************************************************/	        			

	}

	for (int i = motor_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

// 	// servos
// 	if (_num_actuators[1] > 0) {
// 		int servos_idx;

// 		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
// 			int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
// 			float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
// 			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
// 			++actuator_idx_matrix[selected_matrix];
// 			++actuator_idx;
// 		}

// 		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
// 			actuator_servos.control[i] = NAN;
// 		}

// 		_actuator_servos_pub.publish(actuator_servos);
// 		const float tolerance = 1e-5f; // Define a small tolerance value

// 		// if (fabs(actuator_motors.control[motors_idx]) < tolerance && disable_motor_0) {

//     	// 	    PX4_INFO("Detected failure for motor %d", motors_idx);
// 		// }

// 	}
}
/*
void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {
		if (failure_detector_status.fd_motor) {

			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}
*/
int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness.T =");
		effectiveness.T().print();
		PX4_INFO("  minimum =");
		_control_allocation[i]->getActuatorMin().T().print();
		PX4_INFO("  maximum =");
		_control_allocation[i]->getActuatorMax().T().print();
		PX4_INFO("  Configured actuators: %i", _control_allocation[i]->numConfiguredActuators());
	}

	/*if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}*/

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
