/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of fgo_gnss.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 * 
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * 
 *******************************************************/



#ifndef TYPES_H
#define TYPES_H

#include <map>
#include <iostream>

namespace libRSF
{
  /** config types */
  enum class ErrorModelType {Gaussian, DCE, cDCE, SC, DCS, GMM};

  enum class ErrorModelMixtureType {None, MaxMix, SumMix};
  enum class ErrorModelTuningType {None, EM, VBI, iEM, iVBI};

  enum class SolutionType {None, Batch, Smoother, Window, Filter};
//  enum class InitializationType {Zero, Random, Transition, Measurement, GroundTruth};

  /** types of used factors */
  enum class FactorType
  {
    None,
    ConstVal1, ConstVal2, ConstVal3,
    ConstDrift1, ConstDrift2, ConstDrift3,
    BetweenValue1, BetweenValue2, BetweenValue3,
    BetweenPose2, BetweenPose3,
    Range2, Range3, Pseudorange2, Pseudorange3, Pseudorange3Sagnac,
    Odom2, Odom6, Odom4, Odom2Diff, Odom4Local,
    Prior1, Prior2, Prior3, Prior4, Prior9, PriorQuat,
    IMUPretintegration, IMUSimple
  };

  /** more general classification */
  enum class AbstractFactorType
  {
    Ranging, GNSS, ClockModel, Odometry, IMU, Radar, Laser, Vision, MotionModel
  };

  /** types of state data */
  enum class StateType {Other, Point2, Point3, GT2, GT3, Quat, QuatEigen, GM1, GM2, GM3, GM4, GM5, Switch, Val1, Val2, Val3, Sample, Time, Angle, IMUBias};
  enum class StateElement {Timestamp, Mean, Covariance, GMM_StdDev, GMM_Weight, TF, Other};

  enum class SensorType {Range2, Range3,
                         Pseudorange3, Pseudorange2,
                         Odom2, Odom3, Odom2Diff,
                         Point2, Point3,
                         Pose2, Pose3,
                         IMU,
                         Val1,
                         Other};
  enum class SensorElement {Timestamp, Mean, StdDev, Covariance, SatPos, SatElevation, SNR, SatID, WheelBase, TF};

  /** cout enums */
  std::ostream& operator << (std::ostream& Os, const SensorType& Type);
  std::ostream& operator << (std::ostream& Os, const StateType& Type);
  std::ostream& operator << (std::ostream& Os, const FactorType& Type);

  /** dictionary to translate factor types into related sensor types */
  const std::map<FactorType, SensorType> FactorSensorDict =
  {
    {FactorType::Range2, SensorType::Range2},
    {FactorType::Range3, SensorType::Range3},
    {FactorType::BetweenPose2, SensorType::Pose2},
    {FactorType::BetweenPose3, SensorType::Pose3},
    {FactorType::Pseudorange2, SensorType::Pseudorange2},
    {FactorType::Pseudorange3, SensorType::Pseudorange3},
    {FactorType::Pseudorange3Sagnac, SensorType::Pseudorange3},
    {FactorType::IMUPretintegration, SensorType::IMU},
    {FactorType::IMUSimple, SensorType::IMU},
    {FactorType::Odom2, SensorType::Odom2},
    {FactorType::Odom2Diff, SensorType::Odom2Diff},
    {FactorType::Odom4, SensorType::Odom3},
    {FactorType::Odom4Local, SensorType::Odom3},
    {FactorType::Odom6, SensorType::Odom3}
  };

  /** dictionaries translate strings (from files) to enums*/
  const std::map<std::string, FactorType> FactorTypeDict =
  {
    {"const_val1",FactorType::ConstVal1},
    {"const_val2",FactorType::ConstVal2},
    {"const_val3",FactorType::ConstVal3},
    {"const_drift1",FactorType::ConstDrift1},
    {"const_drift2",FactorType::ConstDrift2},
    {"const_drift3",FactorType::ConstDrift3},
    {"between_val1",FactorType::BetweenValue1},
    {"between_val2",FactorType::BetweenValue2},
    {"between_val3",FactorType::BetweenValue3},
    {"between_pose2",FactorType::BetweenPose2},
    {"between_pose3",FactorType::BetweenPose3},
    {"range2",FactorType::Range2},
    {"range3",FactorType::Range3},
    {"pseudorange2",FactorType::Pseudorange2},
    {"pseudorange3",FactorType::Pseudorange3},
    {"pseudorange3_ecef",FactorType::Pseudorange3Sagnac},
    {"odom2",FactorType::Odom2},
    {"odom2diff",FactorType::Odom2Diff},
    {"odom4_ecef",FactorType::Odom4},
    {"odom4",FactorType::Odom4Local},
    {"odom6",FactorType::Odom6},
    {"prior1",FactorType::Prior1},
    {"prior2",FactorType::Prior2},
    {"prior3",FactorType::Prior3},
    {"prior4",FactorType::Prior4},
    {"prior9",FactorType::Prior9},
    {"prior_quat",FactorType::PriorQuat},
    {"imu_pre",FactorType::IMUPretintegration},
    {"imu_simple",FactorType::IMUSimple}
  };

  const std::map<std::string, AbstractFactorType> AbstractFactorTypeDict =
  {
    {"clock",AbstractFactorType::ClockModel},
    {"gnss",AbstractFactorType::GNSS},
    {"imu",AbstractFactorType::IMU},
    {"laser",AbstractFactorType::Laser},
    {"odom",AbstractFactorType::Odometry},
    {"radar",AbstractFactorType::Radar},
    {"uwb",AbstractFactorType::Ranging},
    {"vision",AbstractFactorType::Vision},
    {"motion",AbstractFactorType::MotionModel}
  };

  const std::map<std::string, ErrorModelType> ErrorModelTypeDict =
  {
    {"gauss",ErrorModelType::Gaussian},
    {"sc",ErrorModelType::SC},
    {"dcs",ErrorModelType::DCS},
    {"dce",ErrorModelType::DCE},
    {"cdce",ErrorModelType::cDCE},
    {"gmm",ErrorModelType::GMM}
  };

  const std::map<std::string, ErrorModelMixtureType> ErrorModelMixtureTypeDict =
  {
    {"mm",ErrorModelMixtureType::MaxMix},
    {"sm",ErrorModelMixtureType::SumMix}
  };

  const std::map<std::string, ErrorModelTuningType> ErrorModelTuningTypeDict =
  {
    {"em",ErrorModelTuningType::EM},
    {"em_i",ErrorModelTuningType::iEM},
    {"vbi",ErrorModelTuningType::VBI},
    {"vbi_i",ErrorModelTuningType::iVBI},
    {"none",ErrorModelTuningType::None}
  };

  const std::map<std::string, SolutionType> SolutionTypeDict =
  {
    {"batch",SolutionType::Batch},
    {"smoother",SolutionType::Smoother},
    {"window",SolutionType::Window},
    {"filter",SolutionType::Filter},
    {"none",SolutionType::None}
  };

  /** map ID strings to sensor type */
  const std::map<SensorType, std::string> SensorIDDict =
  {
    {SensorType::Pseudorange2, "pseudorange2"},
    {SensorType::Pseudorange3, "pseudorange3"},
    {SensorType::Range2, "range2"},
    {SensorType::Range3, "range3"},
    {SensorType::IMU, "imu"},
    {SensorType::Odom2, "odom2"},
    {SensorType::Odom2Diff, "odom2diff"},
    {SensorType::Odom3, "odom3"},
    {SensorType::Point2, "point2"},
    {SensorType::Point3, "point3"},
    {SensorType::Pose2, "pose2"},
    {SensorType::Pose3, "pose3"},
    {SensorType::Val1, "val1"},
    {SensorType::Other, "other"}
  };

  const std::map<StateType, std::string> StateTypeDict =
  {
    {StateType::Other, "other"},
    {StateType::Point2, "point2"},
    {StateType::Point3, "point3"},
    {StateType::GT2, "gt2"},
    {StateType::GT3, "gt2"},
    {StateType::Quat, "quaternion"},
    {StateType::QuatEigen, "quaternion_eigen"},
    {StateType::GM1, "gm1"},
    {StateType::GM2, "gm2"},
    {StateType::GM3, "gm3"},
    {StateType::GM4, "gm4"},
    {StateType::GM5, "gm5"},
    {StateType::Switch, "switch"},
    {StateType::Val1, "val1"},
    {StateType::Val1, "val2"},
    {StateType::Val1, "val3"},
    {StateType::Sample, "sample"},
    {StateType::Time, "time"},
    {StateType::Angle, "angle"},
    {StateType::IMUBias, "imu_bias"}
  };

  template <typename DictType>
  bool TranslateSafe(const DictType Dict, typename DictType::key_type const &Original, typename DictType::mapped_type &Translation)
  {
    try
    {
      Translation = Dict.at(Original);
    }
    catch (std::out_of_range& e)
    {
      /** Due to dependency cycle, we can not use PRINT_ERROR() here! */
      std::cerr << "Error in Types.h | Line 217 | TranslateSafe(): Dictionary entry is missing: " << Original;
      return false;
    }
    return true;
  };

  std::string TranslateStateToString(const StateType &State);
}

#endif // TYPES_H
