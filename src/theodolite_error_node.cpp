#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "theodolite_error_ros/theodolite_delta_msg.h"
#include "theodolite_node_msgs/TheodoliteCoordsStamped.h"

using theodolite_delta_msg = theodolite_error_ros::theodolite_delta_msg;

ros::Publisher TS1DeltaPub;
ros::Publisher TS2DeltaPub;
ros::Publisher TS3DeltaPub;

ros::Publisher TS1DeltaAzimuthPub;
ros::Publisher TS2DeltaAzimuthPub;
ros::Publisher TS3DeltaAzimuthPub;

ros::Publisher TS1DeltaElevationPub;
ros::Publisher TS2DeltaElevationPub;
ros::Publisher TS3DeltaElevationPub;

ros::Publisher TS1DeltaDistancePub;
ros::Publisher TS2DeltaDistancePub;
ros::Publisher TS3DeltaDistancePub;

double TS1LastAzimuth;
double TS2LastAzimuth;
double TS3LastAzimuth;

double TS1LastElevation;
double TS2LastElevation;
double TS3LastElevation;

double TS1LastDistance;
double TS2LastDistance;
double TS3LastDistance;

ros::Time TS1LastTime;
ros::Time TS2LastTime;
ros::Time TS3LastTime;

void callback(const theodolite_node_msgs::TheodoliteCoordsStamped &data) {
  theodolite_delta_msg delta_msg;
  delta_msg.header.frame_id = data.header.frame_id;
  delta_msg.header.stamp = data.header.stamp;

  if (data.theodolite_id == 1) {
    if (data.status == 0) {
      ros::Duration deltaTime = data.header.stamp - TS1LastTime;

      if (deltaTime.sec <= 1) {
        delta_msg.delta_azimuth = std::fabs(data.azimuth - TS1LastAzimuth);
        delta_msg.delta_elevation =
            std::fabs(data.elevation - TS1LastElevation);
        delta_msg.delta_distance = std::fabs(data.distance - TS1LastDistance);
        TS1DeltaPub.publish(delta_msg);
      }

      TS1LastAzimuth = data.azimuth;
      TS1LastElevation = data.elevation;
      TS1LastDistance = data.distance;
      TS1LastTime = data.header.stamp;
    }
  } else if (data.theodolite_id == 2) {
    if (data.status == 0) {
      ros::Duration deltaTime = data.header.stamp - TS2LastTime;

      if (deltaTime.sec <= 1) {
        delta_msg.delta_azimuth = std::fabs(data.azimuth - TS2LastAzimuth);
        delta_msg.delta_elevation =
            std::fabs(data.elevation - TS2LastElevation);
        delta_msg.delta_distance = std::fabs(data.distance - TS2LastDistance);
        TS2DeltaPub.publish(delta_msg);
      }

      TS2LastAzimuth = data.azimuth;
      TS2LastElevation = data.elevation;
      TS2LastDistance = data.distance;
      TS2LastTime = data.header.stamp;
    }
  } else if (data.theodolite_id == 3) {
    if (data.status == 0) {
      ros::Duration deltaTime = data.header.stamp - TS3LastTime;

      if (deltaTime.sec <= 1) {
        delta_msg.delta_azimuth = std::fabs(data.azimuth - TS3LastAzimuth);
        delta_msg.delta_elevation =
            std::fabs(data.elevation - TS3LastElevation);
        delta_msg.delta_distance = std::fabs(data.distance - TS3LastDistance);
        TS3DeltaPub.publish(delta_msg);
      }

      TS3LastAzimuth = data.azimuth;
      TS3LastElevation = data.elevation;
      TS3LastDistance = data.distance;
      TS3LastTime = data.header.stamp;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "theodolite_error_node");
  ros::NodeHandle n;

  TS1LastAzimuth = TS2LastAzimuth = TS3LastAzimuth = 0.0;
  TS1LastElevation = TS2LastElevation = TS3LastElevation = 0.0;
  TS1LastDistance = TS2LastDistance = TS3LastDistance = 0.0;
  TS1LastTime = TS2LastTime = TS3LastTime = ros::Time(0);

  ros::Subscriber theodoliteDataSub =
      n.subscribe("/theodolite_master/theodolite_data", 10, callback);

  TS1DeltaPub = n.advertise<theodolite_delta_msg>("TS1_raw_delta", 10);
  TS2DeltaPub = n.advertise<theodolite_delta_msg>("TS2_raw_delta", 10);
  TS3DeltaPub = n.advertise<theodolite_delta_msg>("TS3_raw_delta", 10);

  ros::spin();

  return 0;
}
