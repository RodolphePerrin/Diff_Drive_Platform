/**
* (C) 2020
*
*
*/


/**
 * @file
 *
 * Implementation of the Extended Kalman Filter  algorithm.
 *
 * @author Rodolphe Perrin
 */

#ifndef EKF_H
#define EKF_H

#include <eigen3/Eigen/Dense>
#include <odometry/state.h>

using namespace std;

namespace diff_drive
{

namespace odometry
{
    
class EKF
{
  /** @brief Dimension of process state. */
  static const int n_x_ = State::dimension;

  /** @brief Process model's state covariance type. */
  typedef typename State::Covariance StateCovariance;
    
  /** @brief Process model's transition matrix type. */
  typedef typename State::TransitionMatrix TransitionMatrix;
   
  /** @brief Measurement matrix type. */
  typedef Eigen::Matrix<double, State::dimension , State::dimension> MeasurementMatrix;

  /** @brief Timestamp of latest update. */
  double timestamp_;

  /** @brief Current process state. */
  State state_;

  /** @brief Current process covariance matrix. */
  StateCovariance covariance_;

  /**@brief Current transition matrix. */
  TransitionMatrix transition_;
  
  /**@brief Measurement matrix. */
  MeasurementMatrix H_;


  /**
   * @brief Init the Kalman filter.
   */
    bool init(State &state);
    
  /**
   * @brief Predict the next state and update the covariance matrix.
   */
  void predict(double dt);

  /**
    * @brief Process noise to estimate the process noise covariance matrix.
    */
   void processNoise(double dt);

  /**
   * @brief Update the current state and covariance matrix in response to a reading
   * retrieved from the given sensor.
   */
  template<class Sensor>
  void update(Sensor &sensor);

public:
    /**
   * @brief EKF constructor.
   */
    EKF();
  /**
   * @brief Retrieve a reading from the given sensor, updating the current state
   * and covariance matrix in response.
   */
  template<class Sensor>
  void operator () (double timestamp, Sensor &sensor);

  /**
   * @brief Return the current process state vector.
   */
  inline const State &getState() const
  {
    return state_;
  }

  /**
   * @brief Return the current process covariance matrix.
   */
  inline const StateCovariance &getCovariance() const
  {
    return covariance_;
  }
};

EKF::EKF():
  timestamp_(std::nan(""))
{
  
}

template<class Sensor>
void EKF::operator () (double timestamp, Sensor &sensor)
{
  cout<<"Entering EKF."<<endl;
  if (std::isnan(timestamp_))
  {
    // Initialize state with first measurement.
       //sensor.estimate();
       
     state_ = sensor.estimate();
     init(state_);
     cout<<"EKF Initialized."<<endl;
          
    // Initialize timestamp.
    timestamp_ = timestamp;
    return;
  }

  // Compute elapsed time in seconds.
  double dt = timestamp - timestamp_;
  cout<<dt<<endl;
          
  //Update transition Matrix F as it depends on the time
  state_.F_(0,2) = dt;
  state_.F_(1,3) = dt;
  state_.F(4,5) = dt;

  // Predict new state.
  predict(dt);

  // Update predictions.
  update(sensor);

  // Update timestamp.
  timestamp_ = timestamp;
}

bool EKF::init(State &state)
{
    state_ = state;
    cout<<state_.state_vector_<<endl;
    
    //Initialize state covariance;
    state_.P_ << 1, 0, 0, 0, 0, 0,
                 0, 1, 0, 0, 0, 0,
                 0, 0, 1000, 0, 0, 0,
                 0, 0, 0, 1000, 0, 0,
                 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 0, 0, 1;
    // Initialize transition matrix
    state_.F_ << 1, 0, 0, 0, 0, 0,
                 0, 1, 0, 0, 0, 0,
                 0, 0, 1, 0, 0, 0,
                 0, 0, 0, 1, 0, 0,
                 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 0, 0, 1;
    
    //Uncertainties are initially null.
    state_.Q_ <<0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0;
    //Initialize Measurement matrix
    H_<<1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0;
    
}



void EKF::predict(double dt)
{
    //Process noise covariance Matrix Q
    processNoise(dt);
    
    //Reuse Kalman filter prediction equation.
    state_.state_vector_=state_.F_*state_.state_vector_;
    Eigen::Matrix<double, State::dimension, State::dimension> F_t = state_.F_.transpose();
    state_.P_=state_.F_*state_.P_*F_t + state_.Q_;
    cout<<"First sv:"<<state_.state_vector_<<endl;
}
          
void EKF::processNoise(double dt)
{
  // Update the process noise covariance matrix for a timestep dt.
  // Our motion model uses Gaussian random accelerations in the x and y directions.
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  float dt4over4 = dt4/4.;
  float dt3over2 = dt3/2.;
    state_.Q_ << dt4over4* state_.acc_noise_x,                 0, dt3over2* state_.acc_noise_x,                   0,  0, 0,
                 0, dt4over4* state_.acc_noise_y,                 0, dt3over2* state_.acc_noise_y,                0,  0,
                dt3over2* state_.acc_noise_x,                 0,      dt2* state_.acc_noise_x,                 0,    0,  0,
                0, dt3over2* state_.acc_noise_y,                 0,      dt2* state_.acc_noise_y,                    0, 0,
                0,                             0,                0,                             0,                    0, 0,
                0,                             0,                0,                             0,                    0, 0;
}


template<class Sensor>
void EKF::update(Sensor &sensor)
{
    Eigen::Matrix<double, 6, 6> Identity;
    Identity << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
  // Retrieve latest sensor reading.
  auto reading = sensor.read();
    Eigen::Matrix<double,State::dimension,1> y_ = reading.measurement_ - H_*state_.state_vector_ ;
    Eigen::Matrix<double, State::dimension, State::dimension> S_ = H_*state_.P_*H_.transpose() + sensor.getNoiseMatrix();
    Eigen::Matrix<double, State::dimension, State::dimension> K_ = state_.P_*H_.transpose()*S_.inverse();
  
    //State and Covariance updated beliefs
    state_.state_vector_ = state_.state_vector_ + K_*y_;
    cout<<K_<<endl;
    state_.x_ =  state_.state_vector_(0);
    state_.y_ =  state_.state_vector_(1);
    state_.vx_ =  state_.state_vector_(2);
    state_.vy_ =  state_.state_vector_(3);
    state_.yaw_ = state_.state_vector_(4);
    state_.wz_ = state_.state_vector_(5);
    state_.P_ = (Identity - K_*H_)*state_.P_;
 
}

} //namespace odometry

} // namespace diff_drive

#endif
