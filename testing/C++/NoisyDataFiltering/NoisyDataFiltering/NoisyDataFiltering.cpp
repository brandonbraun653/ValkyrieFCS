#include <iostream>
#include <fstream>
#include <string>

/* Boost Includes */
#include <boost/container/vector.hpp>

/* Kalman Includes */
#include "SquareRootUnscentedKalmanFilter.hpp"

/* Project Includes */
#include "IMUModel.hpp"

typedef float T;

typedef IMU::State<T> State;
typedef IMU::Control<T> Control;
typedef IMU::Measurement<T> Measurement;
typedef IMU::SystemModel<T> SystemModel;
typedef IMU::MeasurementModel<T> MeasurementModel;


const boost::container::vector<std::string> fileNames = {
	"ax_noisy.csv",
	"ay_noisy.csv",
	"az_noisy.csv",
	"gx_noisy.csv",
	"gy_noisy.csv",
	"gz_noisy.csv"
};

int main()
{

	/*----------------------------------
	* Import the noisy accel/gyro data
	*----------------------------------*/
	boost::container::vector<boost::container::vector<double>> inputData, filteredData;
	inputData.resize(fileNames.size());
	filteredData.resize(fileNames.size());

	for (int i = 0; i < fileNames.size(); i++)
	{
		std::ifstream file(fileNames[i]);
		if (!file.is_open()) std::cout << "Error: File Open" << std::endl;

		std::string timeStamp, value;
		while (!file.eof())
		{
			getline(file, timeStamp, ',');
			getline(file, value, '\n');

			inputData[i].push_back(std::stod(value));
		}
	}

	/*----------------------------------
	* Initialize the UKF
	*----------------------------------*/
	//True system state
	State x;
	x.setZero();

	//Control Input
	Control u;
	u.setZero();

	//System
	SystemModel sys;
	
	Eigen::Matrix<T, 6, 6> processNoise;

	T cnst = 0.1f;
	T dnst = 1.0f;
	T enst = 1.0f;

	processNoise <<
		cnst, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, dnst, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, enst, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, cnst, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, dnst, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, enst;

	sys.setCovariance(processNoise);
	//sys.setCovarianceSquareRoot(processNoise);


	//Measurement Model
	MeasurementModel om;

	Eigen::Matrix<T, 6, 6> R;
	R.setZero();
	
	float accelUncertainty = 0.5f;
	float gyroUncertainty = 0.8f;

	R(0, 0) = accelUncertainty * accelUncertainty;
	R(1, 1) = accelUncertainty * accelUncertainty;
	R(2, 2) = accelUncertainty * accelUncertainty;
	R(3, 3) = gyroUncertainty * gyroUncertainty;
	R(4, 4) = gyroUncertainty * gyroUncertainty;
	R(5, 5) = gyroUncertainty * gyroUncertainty;

	om.setCovariance(R);

	//Estimation filter
	Kalman::SquareRootUnscentedKalmanFilter<State> ukf(0.5f, 2.0f, 0.0f);

	ukf.init(x);
	

	/*----------------------------------
	* Run the UKF
	*----------------------------------*/
	const size_t N = inputData[0].size();
	for (size_t i = 0; i < N; i++)
	{

		//Simulate the system
		x = sys.f(x, u);

		//Predict state for current time step 
		auto x_ukf = ukf.predict(sys);

		//Take a measurement given system state
		Measurement meas;
		meas.ax() = inputData[0][i];
		meas.ay() = inputData[1][i];
		meas.az() = inputData[2][i];
		meas.gx() = inputData[3][i];
		meas.gy() = inputData[4][i];
		meas.gz() = inputData[5][i];

		//Update the state equation given measurement
		x_ukf = ukf.update(om, meas);

		filteredData[0].push_back(x_ukf.ax());
		filteredData[1].push_back(x_ukf.ay());
		filteredData[2].push_back(x_ukf.az());
		filteredData[3].push_back(x_ukf.gx());
		filteredData[4].push_back(x_ukf.gy());
		filteredData[5].push_back(x_ukf.gz());
	}

	std::ofstream outputFile;


	/* Write the filtered data to file */
	std::string filename = "filteredData.csv";

	outputFile.open(filename);

	for (int j = 0; j < filteredData[0].size(); j++)
	{
		//Write all values up to the last one
		for (int i = 0; i < filteredData.size() - 1; i++)
			outputFile << filteredData[i][j] << ',';

		//Write the last value 
		outputFile << filteredData[filteredData.size() - 1][j] << "\n";
	}
	
	outputFile.close();

	/* Write the input data to file */
	filename = "inputData.csv";

	outputFile.open(filename);

	for (int j = 0; j < inputData[0].size(); j++)
	{
		//Write all values up to the last one
		for (int i = 0; i < inputData.size() - 1; i++)
			outputFile << inputData[i][j] << ',';

		//Write the last value 
		outputFile << inputData[inputData.size() - 1][j] << "\n";
	}

	outputFile.close();

    return 0;
}

//First, verify build of robot example...then apply to drone problem.