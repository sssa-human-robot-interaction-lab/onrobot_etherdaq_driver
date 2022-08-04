#include <onrobot_etherdaq_driver/onrobot_etherdaq_driver.hpp>

static void SendCommand(SOCKET_HANDLE *socket, uint16 command, uint32 data)
{
	byte request[8];
	*(uint16*)&request[0] = htons(0x1234); 
	*(uint16*)&request[2] = htons(command); 
	*(uint32*)&request[4] = htonl(data); 
	send(*socket, (const char *)request, 8, 0);
	MySleep(5); // Wait a little just to make sure that the command has been processed by Ethernet DAQ
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

	Response r;
	unsigned int i;
	SOCKET_HANDLE socketHandle;		/* Handle to UDP socket used to communicate with Ethernet DAQ. */
	if (argc < 2) {
		ROS_ERROR("Usage: %s IPADDRESS", argv[0] );
		return -1;
	}
	if (Connect(&socketHandle, argv[1], UDP_PORT, SOCK_DGRAM, IPPROTO_UDP) != 0) {
		ROS_ERROR("Could not connect to device...");
		return -1;
	}
  else {
		ROS_INFO("Connected to EtherDAQ");
	}

  ros::Publisher ethdaq_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ethdaq_data",1000);
  
  geometry_msgs::WrenchStamped w;
  w.header.frame_id = "hex_sensor_frame";

	SendCommand(&socketHandle, COMMAND_SPEED, SPEED);
	SendCommand(&socketHandle, COMMAND_FILTER, FILTER);
	SendCommand(&socketHandle, COMMAND_BIAS, BIASING_OFF);
	SendCommand(&socketHandle, COMMAND_START, 0);

	while (r.status == 0 & ros::ok())
  {
		r = Receive(&socketHandle);
		WrenchResponse(&r,&w);
    w.header.stamp = ros::Time::now();
    ethdaq_pub.publish(w);
	}
	
	if (r.status != 0)
	{
		ROS_ERROS("Error code: %d", r.status);
	}

	Close(&socketHandle);
	return 0;
}

