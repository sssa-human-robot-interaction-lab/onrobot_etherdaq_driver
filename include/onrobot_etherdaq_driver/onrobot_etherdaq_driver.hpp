#include <onrobot_etherdaq_driver/onrobot_etherdaq_driver.h>

/* Sleep ms milliseconds */
static void MySleep(unsigned long ms)
{
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}


static int Connect(SOCKET_HANDLE * handle, const char * ipAddress, uint16 port, uint16 type, uint16 proto)
{
	struct sockaddr_in addr;
	struct hostent *he;
	int err;
#ifdef _WIN32
	WSADATA wsaData;
	WORD wVersionRequested;
	wVersionRequested = MAKEWORD(2, 2);
	WSAStartup(wVersionRequested, &wsaData);
	if (GetLastError() != 0) {
		return -1;
	}
#endif

	*handle = socket(AF_INET, type, proto);
#ifdef _WIN32
	if (*handle == INVALID_SOCKET) {
#else
	if (*handle == -1) {
#endif
		ROS_ERROR("Socket could not be opened.");
		return -2;
	}
	he = gethostbyname(ipAddress);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	ROS_INFO("Connecting to EtherDAQ...");
	err = connect(*handle, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		return -3;
	}
	return 0;
}

static void Close(SOCKET_HANDLE * handle)
{
#ifdef _WIN32
	closesocket(*handle);
	WSACleanup();
#else
	close(*handle);
#endif
}