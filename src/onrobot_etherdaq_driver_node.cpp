#include <onrobot_etherdaq_driver/onrobot_etherdaq_driver.hpp>

#include <std_srvs/SetBool.h>

static void SendCommand(SOCKET_HANDLE *socket, uint16 command, uint32 data)
{
	byte request[8];
	*(uint16*)&request[0] = htons(0x1234); 
	*(uint16*)&request[2] = htons(command); 
	*(uint32*)&request[4] = htonl(data); 
	send(*socket, (const char *)request, 8, 0);
	MySleep(5); // Wait a little just to make sure that the command has been processed by Ethernet DAQ
}

static bool BiasingCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response, SOCKET_HANDLE* socket)
{
  if(request.data) SendCommand(socket,COMMAND_BIAS,BIASING_ON);
  else SendCommand(socket,COMMAND_BIAS,BIASING_OFF);
	return true;
}

static Response Receive(SOCKET_HANDLE *socket)
{
	byte inBuffer[36];
	Response response;
	unsigned int uItems = 0;
	recv(*socket, (char *)inBuffer, 36, 0 );
	response.sequenceNumber = ntohl(*(uint32*)&inBuffer[0]);
	response.sampleCounter = ntohl(*(uint32*)&inBuffer[4]);
	response.status = ntohl(*(uint32*)&inBuffer[8]);
	response.fx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.fy = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4])); 
	response.fz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.tx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.ty = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.tz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	return response;
}

static void WrenchResponse(Response *r, geometry_msgs::WrenchStamped *w)
{
	w->wrench.force.x = (double)r->fx / FORCE_DIV;
	w->wrench.force.y = (double)r->fy / FORCE_DIV;
	w->wrench.force.z = (double)r->fz / FORCE_DIV;
	w->wrench.torque.x = (double)r->tx / TORQUE_DIV;
	w->wrench.torque.y = (double)r->ty / TORQUE_DIV;
	w->wrench.torque.z = (double)r->tz / TORQUE_DIV;
}

int main ( int argc, char ** argv ) 
{
  ros::init(argc,argv,"onrobot_etherdaq_driver_node");
  ros::NodeHandle nh;

  std::string ip_address, frame_id;
  int speed, filter;

  ros::param::param<std::string>("~ip_address",ip_address,"192.168.1.1");
  ros::param::param("~speed",speed,2);
  ros::param::param("~filter",filter,4);
  ros::param::param<std::string>("~frame_id",frame_id,"ft_sensor_frame");

  ROS_DEBUG("IP:%s",ip_address.c_str());
  ROS_DEBUG("SPEED:%i",speed);
  ROS_DEBUG("FILTER:%i",filter);
  ROS_DEBUG("FRAME_ID:%s",frame_id.c_str());

	Response r;
	unsigned int i;
	SOCKET_HANDLE socketHandle;		/* Handle to UDP socket used to communicate with Ethernet DAQ. */
	if (Connect(&socketHandle, ip_address.c_str(), UDP_PORT, SOCK_DGRAM, IPPROTO_UDP) != 0) {
		ROS_ERROR("Could not connect to device (IP:%s)",ip_address.c_str());
		return -1;
	}

  ros::ServiceServer biasing_ser = nh.advertiseService<std_srvs::SetBoolRequest,std_srvs::SetBoolResponse>("set_zero",boost::bind(&BiasingCallback, _1, _2, &socketHandle));
  ros::Publisher ethdaq_pub = nh.advertise<geometry_msgs::WrenchStamped>("ethdaq_data",1000);
  
  geometry_msgs::WrenchStamped w;
  w.header.frame_id = frame_id;

	SendCommand(&socketHandle, COMMAND_SPEED, speed);
	SendCommand(&socketHandle, COMMAND_FILTER, filter);
	SendCommand(&socketHandle, COMMAND_BIAS, BIASING_OFF);
	SendCommand(&socketHandle, COMMAND_START, 0);

	while (r.status == 0 && ros::ok())
  {
		r = Receive(&socketHandle);
    if(w.header.seq == 0)ROS_INFO("Connected to EtherDAQ!");
		WrenchResponse(&r,&w);
    w.header.stamp = ros::Time::now();
    ethdaq_pub.publish(w);
    w.header.seq ++;
    ros::spinOnce();
	}
	
	if (r.status != 0)
	{
		ROS_ERROR("Error code: %d", r.status);
	}
	else
	{
		SendCommand(&socketHandle, COMMAND_STOP, 0);
	}

	Close(&socketHandle);
	return 0;
}

