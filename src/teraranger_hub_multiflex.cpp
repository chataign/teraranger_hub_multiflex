/****************************************************************************
 *
 * Copyright (C) 2014 Flavio Fontana & Luis Rodrigues. All rights reserved.
 * Author: Flavio Fontana <fly.fontana@gmail.com>
 * Author: Luis Rodrigues <luis.rodrigues@terabee.com>

 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name Teraranger_hub nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 ****************************************************************************/

#include <string>
#include <sstream>
#include <iomanip>

#include "teraranger_hub_multiflex/teraranger_hub_multiflex.h"

#include <ros/console.h>

namespace teraranger_hub_multiflex
{

Teraranger_hub_multiflex::Teraranger_hub_multiflex()
{
  int queue_size;
  std::string portname;
  double field_of_view, min_range, max_range;
  
  // Get paramters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("single_publisher", single_publisher_, true);
  private_node_handle_.param("publish_laserscan", publish_laserscan_, false);
  private_node_handle_.param("portname", portname, std::string("/dev/ttyACM0"));
  private_node_handle_.param("queue_size", queue_size, 1);
  private_node_handle_.param("field_of_view", field_of_view, 0.2967);
  private_node_handle_.param("min_range", min_range, 0.05);
  private_node_handle_.param("max_range", max_range, 2.0);

  // Publishers
  if ( single_publisher_ )
  {
    range_publisher_ = publish_laserscan_ ?
    	nh_.advertise<sensor_msgs::LaserScan>("teraranger_hub_multiflex", 8):
    	nh_.advertise<sensor_msgs::Range>("teraranger_hub_multiflex", 8);
  }
  else
  {
    for ( int i=0; i<8; ++i ) individual_publishers_[i] = publish_laserscan_ ?
      nh_.advertise<sensor_msgs::LaserScan>("teraranger_hub_multiflex_" + IntToString(i), queue_size ):
      nh_.advertise<sensor_msgs::Range>("teraranger_hub_multiflex_" + IntToString(i), queue_size );
  }

  // Create serial port
  serial_port_ = new SerialPort();

  // Set callback function for the serial ports
  serial_data_callback_function_ = boost::bind(&Teraranger_hub_multiflex::serialDataCallback, this, _1);
  serial_port_->setSerialCallbackFunction(&serial_data_callback_function_);

  // Connect serial port
  if (!serial_port_->connect(portname))
  {
    ros::shutdown();
    return;
  }

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:", ros::this_node::getName().c_str());
  ROS_INFO("[%s] portname: %s", ros::this_node::getName().c_str(), portname.c_str());

  std::string ns = ros::this_node::getNamespace();
  ns = ros::names::clean(ns);
  ROS_INFO("node namespace: [%s]", ns.c_str());

	for(int i=0; i<8; i++)
	{
		  std::string frame_id = "base_range_" + IntToString(i);

		  range_msgs_[i].field_of_view = field_of_view;
		  range_msgs_[i].min_range = min_range;
		  range_msgs_[i].max_range = max_range;
		  range_msgs_[i].radiation_type = sensor_msgs::Range::INFRARED;
		  range_msgs_[i].header.frame_id = ros::names::append(ns, frame_id);

		  scan_msgs_[i].range_min = min_range;
		  scan_msgs_[i].range_max = max_range;
		  scan_msgs_[i].angle_min = -field_of_view/2;
		  scan_msgs_[i].angle_max = +field_of_view/2;
		  scan_msgs_[i].angle_increment = field_of_view;
		  scan_msgs_[i].ranges.resize(2);
		  scan_msgs_[i].header.frame_id = ros::names::append(ns, frame_id);
	}

  // Set operation Mode
 setMode(BINARY_MODE);

  // Initialize all active sensors

  // Dynamic reconfigure
  dyn_param_server_callback_function_ = boost::bind(&Teraranger_hub_multiflex::dynParamCallback, this, _1, _2);
  dyn_param_server_.setCallback(dyn_param_server_callback_function_);
}

Teraranger_hub_multiflex::~Teraranger_hub_multiflex()
{
}

uint8_t Teraranger_hub_multiflex::crc8(uint8_t *p, uint8_t len)
{
  uint16_t i;
  uint16_t crc = 0x0;

  while (len--)
  {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

void Teraranger_hub_multiflex::parseCommand(uint8_t *input_buffer, uint8_t len)
{

	static int seq_ctr = 0;
  int16_t crc = crc8(input_buffer, 19);

	if (crc == input_buffer[19])
	{
		uint8_t bitmask = input_buffer[18];
		uint8_t bit_compare = 1;

		for(int i=0; i<8; i++)
		{
			if ((bitmask & bit_compare) == bit_compare)
			{
				int16_t range_cm;
				range_cm = input_buffer[i*2 + 2] << 8;
				range_cm |= input_buffer[i*2 + 3];
				double range_m = range_cm * 0.001;
				
				ros::Publisher& pub = ( single_publisher_ ? range_publisher_ : individual_publishers_[i] );

				if ( publish_laserscan_ )
				{
					if ( range_m < scan_msgs_[i].range_min || range_m > scan_msgs_[i].range_max )
						range_m = -1.0;

					scan_msgs_[i].ranges.assign( scan_msgs_[i].ranges.size(), range_m );
					scan_msgs_[i].header.stamp = ros::Time::now();
					scan_msgs_[i].header.seq = seq_ctr++;
					pub.publish( scan_msgs_[i] );
				}
				else
				{
					if ( range_m < range_msgs_[i].min_range || range_m > range_msgs_[i].max_range )
						range_m = -1.0;

					range_msgs_[i].range = range_m;
					range_msgs_[i].header.stamp = ros::Time::now();
					range_msgs_[i].header.seq = seq_ctr++;
					pub.publish( range_msgs_[i] );
				}
			}
			else
			{
				ROS_WARN_ONCE("Not all sensors activated set proper bitmask using rosrun rqt_reconfigure rqt_reconfigure");
			}
			bit_compare <<= 1;
		}
	}
	else
	{
	  ROS_ERROR("[%s] crc missmatch", ros::this_node::getName().c_str());
    }

}

std::string Teraranger_hub_multiflex::arrayToString(uint8_t *input_buffer, uint8_t len)
{
	std::ostringstream convert;
	for (int a = 0; a < len; a++) {
		convert << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (int)input_buffer[a];
		convert << std::uppercase << std::hex << " ";
	}
	std::string str = convert.str();
	return str;
}

void Teraranger_hub_multiflex::serialDataCallback(uint8_t single_character)
{
	static uint8_t input_buffer[BUFFER_SIZE];
	static int buffer_ctr = 0;
	static int size_frame = 5;
	static char first_char = 'R';


	if (single_character == 'M' && buffer_ctr == 0)
	{
		size_frame = 20;
		first_char = 'M';
		input_buffer[buffer_ctr] = single_character;
		buffer_ctr++;
	}

	else if (single_character == 'R' && buffer_ctr == 0)
	{
		size_frame = 5;
		first_char = 'R';
		input_buffer[buffer_ctr] = single_character;
		buffer_ctr++;
	}

	else if (first_char == 'R' && buffer_ctr < size_frame)
	{
		input_buffer[buffer_ctr] = single_character;
		buffer_ctr++;

		if (buffer_ctr == size_frame)
		{
			std::string str = arrayToString(input_buffer,size_frame);
			ROS_DEBUG_STREAM("Respond frame reveived... : " << str);
			// reset
			buffer_ctr = 0;
			// clear struct
			bzero(&input_buffer, BUFFER_SIZE);
		}
	}
	else if (first_char == 'M' && buffer_ctr < size_frame)
	{
		input_buffer[buffer_ctr] = single_character;
		buffer_ctr++;
		if (buffer_ctr == size_frame)
		{
			std::string str = arrayToString(input_buffer,size_frame);
			ROS_DEBUG_STREAM("Frame reveived... : " << str);

			parseCommand(input_buffer,19);
			// reset
			buffer_ctr = 0;
			// clear struct
			bzero(&input_buffer, BUFFER_SIZE);

		}
	}
	else
	{
		ROS_DEBUG("Received uknown character %x", single_character);
		// reset
		buffer_ctr = 0;
		// clear struct
		bzero(&input_buffer, BUFFER_SIZE);
	}
}
void Teraranger_hub_multiflex::setMode(const char *c)
{
 serial_port_->sendChar(c, 4);
}

void Teraranger_hub_multiflex::setSensorBitMask(int *sensor_bit_mask_ptr)
{

 uint8_t bit_mask_hex=0x00;
 for (int i=0; i<8; i++)
 {
	 bit_mask_hex |= *(sensor_bit_mask_ptr +7-i) << (7-i);
 }

 // calculate crc

 uint8_t command[4] = {0x00, 0x52, 0x03, bit_mask_hex};
 int8_t crc = crc8(command, 4);

 //send command
 char full_command[5] = {0x00, 0x52, 0x03,bit_mask_hex,crc};


 serial_port_->sendChar(full_command, 5);
}

void Teraranger_hub_multiflex::dynParamCallback(const teraranger_hub_multiflex::Teraranger_hub_multiflexConfig &config, uint32_t level)
{

  if (level == 1)
  {
	  sensor_bit_mask[0] = config.Sensor_0?1:0;
	  sensor_bit_mask[1] = config.Sensor_1?1:0;
	  sensor_bit_mask[2] = config.Sensor_2?1:0;
	  sensor_bit_mask[3] = config.Sensor_3?1:0;
	  sensor_bit_mask[4] = config.Sensor_4?1:0;
	  sensor_bit_mask[5] = config.Sensor_5?1:0;
	  sensor_bit_mask[6] = config.Sensor_6?1:0;
	  sensor_bit_mask[7] = config.Sensor_7?1:0;

	  sensor_bit_mask_ptr = sensor_bit_mask;

	  setSensorBitMask(sensor_bit_mask_ptr);

	  for(int i = 0; i<8; i++)
	  {
		  ROS_INFO("Sensor %d is set to %d",i,sensor_bit_mask[i]);
	  }
  }

  // else if (level == 0)
  // {
  //  if (config.Mode == teraranger_hub_multiflex::Teraranger_hub_multiflex_Fast)
  //  {
  //    setMode(FAST_MODE);
  // ROS_INFO("Fast mode set");
  //  }
  //
  //  if (config.Mode == teraranger_hub_multiflex::Teraranger_hub_multiflex_Precise)
  //  {
  //    setMode(PRECISE_MODE);
  // ROS_INFO("Precise mode set");
  //  }
  //
  //  if (config.Mode == teraranger_hub_multiflex::Teraranger_hub_multiflex_LongRange)
  //  {
  //    setMode(LONG_RANGE_MODE);
  // ROS_INFO("Long range mode set");
  //  }
  // }
  else
  {
	  ROS_DEBUG("Dynamic reconfigure, got %d", level);
  }

}

std::string Teraranger_hub_multiflex::IntToString( int number )
{
	std::ostringstream oss;
	oss << number;
	return oss.str();
}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Teraranger_hub_multiflex");
  teraranger_hub_multiflex::Teraranger_hub_multiflex multiflex;
  ros::spin();

  return 0;
}
