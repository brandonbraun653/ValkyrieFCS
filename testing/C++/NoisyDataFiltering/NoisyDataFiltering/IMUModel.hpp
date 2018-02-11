#pragma once
#ifndef IMU_MODEL_HPP
#define IMU_MODEL_HPP

#include "Matrix.hpp"
#include "LinearizedMeasurementModel.hpp"
#include "LinearizedSystemModel.hpp"

namespace IMU
{ 


template<typename T>
class State : public Kalman::Vector<T,6>
{
public:
	KALMAN_VECTOR(State, T, 6)

	static constexpr size_t AX = 0;
	static constexpr size_t AY = 1;
	static constexpr size_t AZ = 2;
	static constexpr size_t GX = 3;
	static constexpr size_t GY = 4;
	static constexpr size_t GZ = 5;

	T ax() const { return (*this)[AX]; }
	T ay() const { return (*this)[AY]; }
	T az() const { return (*this)[AZ]; }
	T gx() const { return (*this)[GX]; }
	T gy() const { return (*this)[GY]; }
	T gz() const { return (*this)[GZ]; }

	T& ax() { return (*this)[AX]; }
	T& ay() { return (*this)[AY]; }
	T& az() { return (*this)[AZ]; }
	T& gx() { return (*this)[GX]; }
	T& gy() { return (*this)[GY]; }
	T& gz() { return (*this)[GZ]; }
};

template<typename T>
class Control : public Kalman::Vector<T, 1>
{
public:
	KALMAN_VECTOR(Control, T, 1)

	//! Velocity
	static constexpr size_t V = 0;

	T v()       const { return (*this)[V]; }
	T& v() { return (*this)[V]; }
	
};


template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
	typedef State<T> S;
	typedef Control<T> C;

	S f(const S& x, const C& u) const
	{
		//Currently only trying to predict the actual accelerometer/gyroscope data, so
		//the measurement model is an identity matrix.
		S x_ = x;
		return x_;
	}
};


template<typename T>
class Measurement : public Kalman::Vector<T, 6>
{
public:
	KALMAN_VECTOR(Measurement, T, 6)

	static constexpr size_t AX = 0;
	static constexpr size_t AY = 1;
	static constexpr size_t AZ = 2;
	static constexpr size_t GX = 3;
	static constexpr size_t GY = 4;
	static constexpr size_t GZ = 5;

	T ax() const { return (*this)[AX]; }
	T ay() const { return (*this)[AY]; }
	T az() const { return (*this)[AZ]; }
	T gx() const { return (*this)[GX]; }
	T gy() const { return (*this)[GY]; }
	T gz() const { return (*this)[GZ]; }

	T& ax() { return (*this)[AX]; }
	T& ay() { return (*this)[AY]; }
	T& az() { return (*this)[AZ]; }
	T& gx() { return (*this)[GX]; }
	T& gy() { return (*this)[GY]; }
	T& gz() { return (*this)[GZ]; }
};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::MeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
	//! State type shortcut definition
	typedef State<T> S;

	//! Measurement type shortcut definition
	typedef Measurement<T> M;

	MeasurementModel()
	{
		// Setup jacobians. As these are static, we can define them once
		// and do not need to update them dynamically
		//this->H.setIdentity();
		//this->V.setIdentity();

		
	}

	/**
	* @brief Definition of (possibly non-linear) measurement function
	*
	* This function maps the system state to the measurement that is expected
	* to be received from the sensor assuming the system is currently in the
	* estimated state.
	*
	* @param [in] x The system state in current time-step
	* @returns The (predicted) sensor measurement for the system state
	*/
	M h(const S& x) const
	{
		M measurement;

		//This is simply the identity matrix
		measurement.ax() = x.ax();
		measurement.ay() = x.ay();
		measurement.az() = x.az();
		measurement.gx() = x.gx();
		measurement.gy() = x.gy();
		measurement.gz() = x.gz();

		return measurement;
	}
};

}
#endif 