/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <mocap_vicon/ViconDriver.h>

#include <mocap_vicon/Markers.h>
#include <mocap_vicon/Marker.h>
#include <mocap_vicon/Devices.h>
#include <mocap_vicon/Device.h>
#include <mocap_vicon/DeviceOutput.h>
#include <mocap_vicon/Sample.h>

namespace mocap {

bool ViconDriver::init() {

  nh.param("server_address", server_address, string("alkaline2"));
  nh.param("model_list", model_list, vector<string>(0));
  nh.param("frame_rate", frame_rate, 100);
  nh.param("max_accel", max_accel, 10.0);
  nh.param("publish_tf", publish_tf, true);
  nh.param("fixed_frame_id", fixed_frame_id, string("mocap"));

  unlabeled_markers_publisher = nh.advertise<mocap_vicon::Markers>("/vicon/unlabeled_markers", 10);
  labeled_markers_publisher = nh.advertise<mocap_vicon::Markers>("/vicon/labeled_markers", 10);
  devices_publisher = nh.advertise<mocap_vicon::Devices>("/vicon/devices", 10);

  frame_interval = 1.0 / static_cast<double>(frame_rate);
  double& dt = frame_interval;
  process_noise.topLeftCorner<6, 6>() =
    0.5*Matrix<double, 6, 6>::Identity()*dt*dt*max_accel;
  process_noise.bottomRightCorner<6, 6>() =
    Matrix<double, 6, 6>::Identity()*dt*max_accel;
  process_noise *= process_noise; // Make it a covariance
  measurement_noise =
    Matrix<double, 6, 6>::Identity()*1e-3;
  measurement_noise *= measurement_noise; // Make it a covariance
  model_set.insert(model_list.begin(), model_list.end());

  timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;

  // Connect to the server
  ROS_INFO("Connecting to Vicon Datastream server at %s", server_address.c_str());
  bool is_connected = false;
  for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
    client->Connect(server_address);
    if(client->IsConnected().Connected) {
      is_connected = true;
      break;
    }
    else
      nanosleep(&ts_sleep, NULL);
  }

  // Report if cannot connect
  if (!is_connected) {
    ROS_WARN("Cannot Connect to Vicon server at %s", server_address.c_str());
    return false;
  }

  // Configure the connection
  ROS_INFO("Successfully Connect to Vicon server at %s", server_address.c_str());
  client->SetStreamMode(ViconSDK::StreamMode::ClientPull);
  client->SetAxisMapping(ViconSDK::Direction::Forward,
      ViconSDK::Direction::Left, ViconSDK::Direction::Up);
  client->EnableSegmentData();
  client->EnableUnlabeledMarkerData();
  client->EnableMarkerData();
  client->EnableDeviceData();
  if(!client->IsSegmentDataEnabled().Enabled) {
    ROS_WARN("Segment data cannot be enabled.");
    return false;
  }
  ROS_INFO("Successfully configure Vicon server at %s", server_address.c_str());

  // Need to wait for some time after enabling data else you get junk frames
  //struct timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;
  nanosleep(&ts_sleep, NULL);

  return true;
}

void ViconDriver::run() {
  ViconSDK::Result::Enum result = client->GetFrame().Result;
  if (result != ViconSDK::Result::Success)
    return;
  handleFrame();
  handleDevices();
  return;
}

void ViconDriver::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server at "
      << server_address);
  client->Disconnect();
  return;
}

void ViconDriver::handleFrame() {
  int body_count = client->GetSubjectCount().SubjectCount;
  // Assign each subject with a thread
  vector<boost::thread> subject_threads;
  subject_threads.reserve(body_count);

  handleUnlabeledMarkers();
  handleLabeledMarkers();
  for (int i = 0; i< body_count; ++i) {
    string subject_name =
      client->GetSubjectName(i).SubjectName;

    // Process the subject if required
    if (model_set.empty() || model_set.count(subject_name)) {
      // Create a new subject if it does not exist
      if (subjects.find(subject_name) == subjects.end()) {
        subjects[subject_name] = Subject::SubjectPtr(
            new Subject(&nh, subject_name, fixed_frame_id));
        subjects[subject_name]->setParameters(
            process_noise, measurement_noise, frame_rate);
      }
      // Handle the subject in a different thread
      subject_threads.emplace_back(&ViconDriver::handleSubject, this, i);
      //handleSubject(i);
    }
  }

  // Wait for all the threads to stop
  for(auto &thread : subject_threads) {
    thread.join();
  }

  // Send out warnings
  for (auto it = subjects.begin();
      it != subjects.end(); ++it) {
    Subject::Status status = it->second->getStatus();
    if (status == Subject::LOST)
      ROS_WARN_THROTTLE(1, "Lose track of subject %s", (it->first).c_str());
    else if (status == Subject::INITIALIZING)
      ROS_WARN("Initialize subject %s", (it->first).c_str());
  }

  return;
}

void ViconDriver::handleSubject(const int& sub_idx) {
  boost::unique_lock<boost::shared_mutex> write_lock(mtx);
  // We assume each subject has only one segment
  string subject_name = client->GetSubjectName(sub_idx).SubjectName;
  string segment_name = client->GetSegmentName(subject_name, 0).SegmentName;
  // Get the pose for the subject
  ViconSDK::Output_GetSegmentGlobalTranslation trans =
      client->GetSegmentGlobalTranslation(subject_name, segment_name);
  ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat =
      client->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
  write_lock.unlock();

  //boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  if(trans.Result != ViconSDK::Result::Success ||
     quat.Result != ViconSDK::Result::Success ||
     trans.Occluded || quat.Occluded) {
    subjects[subject_name]->disable();
    return;
  }

  // Convert the msgs to Eigen type
  Eigen::Quaterniond m_att(0,
      0, 0, 0);

  //Eigen::Quaterniond m_att(quat.Rotation[3],
  //    quat.Rotation[0], quat.Rotation[1], quat.Rotation[2]);
  Eigen::Vector3d m_pos(trans.Translation[0]/1000,
      trans.Translation[1]/1000, trans.Translation[2]/1000);

  // Re-enable the object if it is lost previously
  if (subjects[subject_name]->getStatus() == Subject::LOST) {
    subjects[subject_name]->enable();
  }

  // Feed the new measurement to the subject
  double time = ros::Time::now().toSec();
  subjects[subject_name]->processNewMeasurement(time, m_att, m_pos);
  //read_lock.unlock();

  // Publish tf if requred
  if (publish_tf &&
      subjects[subject_name]->getStatus() == Subject::TRACKED) {

    Eigen::Quaterniond att = subjects[subject_name]->getAttitude();
    Vector3d pos = subjects[subject_name]->getPosition();
    tf::Quaternion att_tf;
    //Eigen::Quaterniond att = Eigen::Quaterniond(0.707,0,0.707,0);
    tf::Vector3 pos_tf;
    tf::quaternionEigenToTF(att, att_tf);
    tf::vectorEigenToTF(pos, pos_tf);

    tf::StampedTransform stamped_transform =
      tf::StampedTransform(tf::Transform(att_tf, pos_tf),
        ros::Time::now(), fixed_frame_id, subject_name);
    write_lock.lock();
    tf_publisher.sendTransform(stamped_transform);
    write_lock.unlock();
  }

  return;
}

void ViconDriver::handleUnlabeledMarkers(){
  mocap_vicon::Markers markers_msg;

  ros::Duration vicon_latency(client->GetLatencyTotal().Total);
  ros::Time frame_time;

  now_time = ros::Time::now();

  frame_time = now_time - vicon_latency;

  frame_num = client->GetFrameNumber().FrameNumber;

  markers_msg.header.stamp = frame_time;
  markers_msg.frame_number = frame_num;

  unsigned int unlabeled_marker_count = client->GetUnlabeledMarkerCount().MarkerCount;
  for( unsigned int marker_index = 0 ; marker_index < unlabeled_marker_count ; ++marker_index )
  {
    // Get the global marker translation
    mocap_vicon::Marker this_marker;
    this_marker.translation.x = client->GetUnlabeledMarkerGlobalTranslation(marker_index).Translation[0];
    this_marker.translation.y = client->GetUnlabeledMarkerGlobalTranslation(marker_index).Translation[1];
    this_marker.translation.z = client->GetUnlabeledMarkerGlobalTranslation(marker_index).Translation[2];
    this_marker.occluded = false;
    markers_msg.markers.push_back(this_marker);
  }

  unlabeled_markers_publisher.publish(markers_msg);
}

void ViconDriver::handleLabeledMarkers(){
  mocap_vicon::Markers markers_msg;

  ros::Duration vicon_latency(client->GetLatencyTotal().Total);
  ros::Time frame_time;

  now_time = ros::Time::now();

  frame_time = now_time - vicon_latency;

  frame_num = client->GetFrameNumber().FrameNumber;

  markers_msg.header.stamp = frame_time;
  markers_msg.frame_number = frame_num;

  unsigned int SubjectCount = client->GetSubjectCount().SubjectCount;
  for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
  {
    std::string SubjectName = client->GetSubjectName( SubjectIndex ).SubjectName;
    unsigned int labeled_marker_count = client->GetMarkerCount( SubjectName ).MarkerCount;
    for( unsigned int marker_index = 0 ; marker_index < labeled_marker_count ; ++marker_index )
    {
      // Get the global marker translation
      mocap_vicon::Marker this_marker;
      std::string MarkerName = client->GetMarkerName( SubjectName, marker_index ).MarkerName;
      this_marker.marker_name = MarkerName;
      this_marker.subject_name = SubjectName;
      this_marker.segment_name = client->GetMarkerParentName( SubjectName, MarkerName ).SegmentName;
      this_marker.translation.x = client->GetMarkerGlobalTranslation(SubjectName, MarkerName).Translation[0];
      this_marker.translation.y = client->GetMarkerGlobalTranslation(SubjectName, MarkerName).Translation[1];
      this_marker.translation.z = client->GetMarkerGlobalTranslation(SubjectName, MarkerName).Translation[2];
      this_marker.occluded = false;
      markers_msg.markers.push_back(this_marker);
    }

  }

  labeled_markers_publisher.publish(markers_msg);
}

void ViconDriver::handleDevices(){
  mocap_vicon::Devices devices_msg;

  ros::Duration vicon_latency(client->GetLatencyTotal().Total);
  ros::Time frame_time;

  now_time = ros::Time::now();

  frame_time = now_time - vicon_latency;

  frame_num = client->GetFrameNumber().FrameNumber;

  devices_msg.header.stamp = frame_time;
  devices_msg.frame_number = frame_num;

  unsigned int device_count = client->GetDeviceCount().DeviceCount;
  for(unsigned int device_index = 0; device_index < device_count; device_index++)
  {
    mocap_vicon::Device this_device;
    this_device.id = device_index;
    this_device.name = client->GetDeviceName(device_index).DeviceName;
    this_device.type = adapt(client->GetDeviceName(device_index).DeviceType);
    unsigned int output_device_count = client->GetDeviceOutputCount(this_device.name).DeviceOutputCount;
    for(unsigned int output_index = 0; output_index < output_device_count; output_index++){
      mocap_vicon::DeviceOutput this_device_output;
      this_device_output.id = output_index;
      this_device_output.name = client->GetDeviceOutputName(this_device.name, output_index).DeviceOutputName;
      this_device_output.unit = adapt(client->GetDeviceOutputName(this_device.name, output_index).DeviceOutputUnit);
      unsigned int sample_count = client->GetDeviceOutputSubsamples(this_device.name,this_device_output.name).DeviceOutputSubsamples;
      for(unsigned int sample_index = 0; sample_index < sample_count; sample_index ++){
        mocap_vicon::Sample this_sample;
        this_sample.id = sample_index;
        this_sample.value = client->GetDeviceOutputValue(this_device.name,this_device_output.name,sample_index).Value;
        this_sample.occluded = client->GetDeviceOutputValue(this_device.name,this_device_output.name,sample_index).Occluded;

        this_device_output.samples.push_back(this_sample);
      }
      this_device.deviceOutputs.push_back(this_device_output);
    }
    devices_msg.devices.push_back(this_device);
  }

  devices_publisher.publish(devices_msg);
}

string ViconDriver::adapt( const DeviceType::Enum i_DeviceType )
{
  switch( i_DeviceType )
  {
    case DeviceType::ForcePlate:
      return "ForcePlate";
    case DeviceType::Unknown:
    default:
      return "Unknown";
  }
}

string ViconDriver::adapt( const Unit::Enum i_Unit )
{
  switch( i_Unit )
  {
    case Unit::Meter:
      return "Meter";
    case Unit::Volt:
      return "Volt";
    case Unit::NewtonMeter:
      return "NewtonMeter";
    case Unit::Newton:
      return "Newton";
    case Unit::Kilogram:
      return "Kilogram";
    case Unit::Second:
      return "Second";
    case Unit::Ampere:
      return "Ampere";
    case Unit::Kelvin:
      return "Kelvin";
    case Unit::Mole:
      return "Mole";
    case Unit::Candela:
      return "Candela";
    case Unit::Radian:
      return "Radian";
    case Unit::Steradian:
      return "Steradian";
    case Unit::MeterSquared:
      return "MeterSquared";
    case Unit::MeterCubed:
      return "MeterCubed";
    case Unit::MeterPerSecond:
      return "MeterPerSecond";
    case Unit::MeterPerSecondSquared:
      return "MeterPerSecondSquared";
    case Unit::RadianPerSecond:
      return "RadianPerSecond";
    case Unit::RadianPerSecondSquared:
      return "RadianPerSecondSquared";
    case Unit::Hertz:
      return "Hertz";
    case Unit::Joule:
      return "Joule";
    case Unit::Watt:
      return "Watt";
    case Unit::Pascal:
      return "Pascal";
    case Unit::Lumen:
      return "Lumen";
    case Unit::Lux:
      return "Lux";
    case Unit::Coulomb:
      return "Coulomb";
    case Unit::Ohm:
      return "Ohm";
    case Unit::Farad:
      return "Farad";
    case Unit::Weber:
      return "Weber";
    case Unit::Tesla:
      return "Tesla";
    case Unit::Henry:
      return "Henry";
    case Unit::Siemens:
      return "Siemens";
    case Unit::Becquerel:
      return "Becquerel";
    case Unit::Gray:
      return "Gray";
    case Unit::Sievert:
      return "Sievert";
    case Unit::Katal:
      return "Katal";

    case Unit::Unknown:
    default:
      return "Unknown";
  }
}

}
