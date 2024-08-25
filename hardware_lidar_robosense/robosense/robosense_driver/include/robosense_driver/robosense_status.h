#ifndef ROBOSENSE_STATUS_H_
#define ROBOSENSE_STATUS_H_ 
/*
* These code defined wanshannnt hardware sttaus for hardware monitor
* Each bit mean different status, report to application
*/
namespace hardware{
	enum class HardwareMonitor : unsigned int{
		LIDARBOX_SOCKET_CREAT_FAILED	= 0x001,
		LIDARBOX_SOCKET_SEND_FAILED		= 0x002,
		LIDARBOX_SOCKET_CONNECT_FAILED	= 0x003,
		LIDARBOX_SOCKET_RECV_FAILED		= 0x004,
		LIDARBOX_SOCKET_DATA_INVALID	= 0x00b,
		LIDARBOX_QUEEN_PUSH_FAILED		= 0x005,
		LIDARBOX_GPGGA_DATA_INVALID		= 0x006,
		LIDARBOX_GPRMC_DATA_INVALID		= 0x007,
		LIDARBOX_IMU_DATA_DELAY_WARN	= 0x008,
		LIDARBOX_IMU_DATA_DELAY_ERROR	= 0x009,
		LIDARBOX_GPS_LOC_FAILED	    	= 0x00a,
		LIDARBOX_SOCKET_ISSET_FAILED	= 0x00c,
		ROBOSENSE_MODEL_TYPE_UNKNOWN	= 0x00d,
		ROBOSENSE_CUTANGLE_OUT_RANGE	= 0x00d,
		ROBOSENSE_SOCKET_CREAT_FAILED	= 0x00e,
		ROBOSENSE_SOCKET_BIND_FAILED	= 0x00f,
		ROBOSENSE_SOCKET_NON_BLOCK		= 0x010,
		ROBOSENSE_SOCKET_POLL_ERROR		= 0x011,
		ROBOSENSE_SOCKET_POLL_TIMEOUT	= 0x012,
		ROBOSENSE_SOCKET_POLL_DEV_ERR	= 0x013,
		ROBOSENSE_SOCKET_RECV_FAILDED	= 0x014,
		ROBOSENSE_PCAP_READ_FAILDED		= 0x015,
		ROBOSENSE_UDP_DATA_TIMEBACK		= 0x016,
		
	
	};
}
#endif