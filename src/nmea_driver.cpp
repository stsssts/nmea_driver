#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <nmea_msgs/Gprmc.h>
#include <nmea_msgs/Gpgga.h>
#include <nmea_msgs/Gpgsa.h>
#include <nmea_msgs/Gpgsv.h>
#include <nmea_msgs/GpgsvSatellite.h>


class NMEADevice
{
public:
  NMEADevice(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");
    std::string addr = pnh.param<std::string>("port", "/dev/ttyUSB0");

    _device_fd = open(addr.c_str(), O_RDONLY | O_NOCTTY);
    if (_device_fd == -1)
    {
      ROS_FATAL("Device %s is unavailible", addr.c_str());
      throw std::exception();
    }

    _configureDevice();
    _readLine(); // wipe input

    _hdt_publisher = nh.advertise<std_msgs::Float64>("hdt", 1);
    _rmc_publisher = nh.advertise<nmea_msgs::Gprmc>("gprmc", 1);
    _gga_publisher = nh.advertise<nmea_msgs::Gpgga>("gpgga", 1);
    _gsa_publisher = nh.advertise<nmea_msgs::Gpgsa>("gpgsa", 1);
    _gsv_publisher = nh.advertise<nmea_msgs::Gpgsv>("gpgsv", 1);
  }

  ~NMEADevice()
  {
    close(_device_fd);
  }

  void update()
  {
    _readLine();
    if (_checkResponse())
      _parseResponce();
  }

private:
  int _device_fd;
  char buffer[82+1]; // maximum sentence length according to NMEA specification + \0

  ros::Publisher _hdt_publisher;
  ros::Publisher _rmc_publisher;
  ros::Publisher _gga_publisher;
  ros::Publisher _gsa_publisher;
  ros::Publisher _gsv_publisher;

  void _configureDevice()
  {
    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(_device_fd, &tty) != 0)
    {
      ROS_FATAL("Error %i from tcgetattr: %s", errno, strerror(errno));
      throw std::exception();
    }

    cfsetospeed(&tty, (speed_t)B115200);
    cfsetispeed(&tty, (speed_t)B115200);

    tty.c_cflag &= ~PARENB;        // 8N1
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN] = 1;            // read doesn't block
    tty.c_cc[VTIME] = 5;           // 0.5 secs read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
    cfmakeraw(&tty);               // make raw

    tcflush(_device_fd, TCIFLUSH);
    if (tcsetattr(_device_fd, TCSANOW, &tty) != 0)
    {
      ROS_FATAL("Error %i from tcsetattr", errno);
      throw std::exception();
    }
  }

  bool _readLine()
  {
    int n = 1;
    int spot = 0;
    char char_buf = '\0';

    memset(buffer, '\0', 82);
    while (n > 0)
    {
      n = read(_device_fd, &char_buf, 1);
      sprintf(&buffer[spot++], "%c", char_buf);

      if (buffer[spot-1] == '\n' &&
          buffer[spot-2] == '\r')
        return true;
    }
  }

  bool _checkResponse()
  {
    char *c = buffer;
    if (*c != '$')
    {
      ROS_ERROR("First char is not equal to $");
      ROS_ERROR_STREAM(buffer);
      return false;
    }

    uint8_t checksum = 0;
    while (*(c+1) != '*')
      checksum ^= *(++c);

    char str_checksum[3];
    sprintf(str_checksum, "%02X", checksum);

    if (strncmp(c+2, str_checksum, 2) == 0)
      return true;
    else
    {
      ROS_ERROR("Message checksum is incorrect, calculated: %s", str_checksum);
      ROS_ERROR_STREAM(buffer);
      return false;
    }
  }

  void _parseResponce()
  {
    char *c = buffer+1;

    if (strncmp(c, "GP", 2) == 0)
    {
      if (strncmp(c+2, "RMC", 3) == 0)
        _publishRMC();
      if (strncmp(c+2, "GGA", 3) == 0)
        _publishGGA();
      else if (strncmp(c+2, "GSA", 3) == 0)
        _publishGSA();
      else if (strncmp(c+2, "GSV", 3) == 0)
        _publishGSV();
      else if (strncmp(c+2, "HDT", 3) == 0)
        _publishHDT();
    }
  }

  std::vector<std::string> _splitSentence(const std::string& s, char delimiter)
  {
    std::vector<std::string> tokens;
    std::istringstream tokenStream(s);

    for (std::string token; std::getline(tokenStream, token, delimiter);)
    {
      if (!token.empty())
        tokens.push_back(token);
      else
        tokens.push_back("0");
    }

    if (s.back() == ',')
      tokens.push_back("0");

    return tokens;
  }

  void _publishRMC()
  {
    // $GPRMC,000101.20,V,0000.0000000,N,00000.0000000,E,0.00,0.00,,,,N*44

    std::string sentence = _splitSentence(std::string(buffer+7), '*')[0];
    std::vector<std::string> tokens = _splitSentence(sentence, ',');

    nmea_msgs::Gprmc msg;
    msg.header.stamp = ros::Time::now();
    msg.message_id = "RMC";
    msg.utc_seconds = std::stod(tokens[0]);
    msg.position_status = tokens[1];
    msg.lat = std::stod(tokens[2]);
    msg.lat_dir = tokens[3];
    msg.lon = std::stod(tokens[4]);
    msg.lon_dir = tokens[5];
    msg.speed = std::stof(tokens[6]);
    msg.track = std::stof(tokens[7]);
    msg.date = tokens[8];
    msg.mag_var = std::stof(tokens[9]);
    msg.mag_var_direction = tokens[10];
    msg.mode_indicator = tokens[11];

    _rmc_publisher.publish(msg);
  }

  void _publishGGA()
  {
    // $GPGGA,144148.88,0000.0000000,N,00000.0000000,E,0,00,,-17.162,M,17.162,M,,*52

    std::string sentence = _splitSentence(std::string(buffer + 7), '*')[0];
    std::vector<std::string> tokens = _splitSentence(sentence, ',');

    nmea_msgs::Gpgga msg;
    msg.header.stamp = ros::Time::now();
    msg.message_id = "GGA";
    msg.utc_seconds = std::stod(tokens[0]);
    msg.lat = std::stod(tokens[1]);
    msg.lat_dir = tokens[2];
    msg.lon = std::stod(tokens[3]);
    msg.lon_dir = tokens[4];
    msg.gps_qual = std::stoi(tokens[5]);
    msg.num_sats = std::stoi(tokens[6]);
    msg.hdop = std::stof(tokens[7]);
    msg.alt = std::stof(tokens[8]);
    msg.altitude_units = tokens[9];
    msg.undulation = std::stof(tokens[10]);
    msg.undulation_units = tokens[11];
    msg.diff_age = std::stoi(tokens[12]);
    msg.station_id = tokens[13];

    _gga_publisher.publish(msg);
  }

  void _publishGSA()
  {
    // $GPGSA,A,1,,,,,,,,,,,,,,,,*32
    std::string sentence = _splitSentence(std::string(buffer +7), '*')[0];
    std::vector<std::string> tokens = _splitSentence(sentence, ',');

    nmea_msgs::Gpgsa msg;
    msg.header.stamp = ros::Time::now();
    msg.message_id = "GSA";
    msg.auto_manual_mode = tokens[0];
    msg.fix_mode = std::stoi(tokens[1]);

    for (int i = 0; i < 12; ++i)
      msg.sv_ids.push_back(std::stoi(tokens[i+2]));

    msg.pdop = std::stof(tokens[14]);
    msg.hdop = std::stof(tokens[15]);
    msg.vdop = std::stof(tokens[16]);

    _gsa_publisher.publish(msg);
  }

  void _publishGSV()
  {
    // $GLGSV,1,1,0,,,,,*79
    std::string sentence = _splitSentence(std::string(buffer + 7), '*')[0];
    std::vector<std::string> tokens = _splitSentence(sentence, ',');

    nmea_msgs::Gpgsv msg;
    msg.header.stamp = ros::Time::now();
    msg.message_id = "GSV";
    msg.n_msgs = std::stoi(tokens[0]);
    msg.msg_number = std::stoi(tokens[1]);
    msg.n_satellites = std::stoi(tokens[2]);

    for (int i = 0; i < msg.n_msgs; ++i)
    {
      nmea_msgs::GpgsvSatellite sat;
      sat.prn = std::stoi(tokens[4*i + 3]);
      sat.elevation = std::stoi(tokens[4*i + 4]);
      sat.azimuth = std::stoi(tokens[4*i + 5]);
      sat.snr = std::stoi(tokens[4*i + 6]);

      msg.satellites.push_back(sat);
    }

    _gsv_publisher.publish(msg);
  }

  void _publishHDT()
  {
    std::string sentence = _splitSentence(std::string(buffer + 7), '*')[0];
    std::vector<std::string> tokens = _splitSentence(sentence, ',');

    if (!tokens[0].empty())
    {
      std_msgs::Float64 msg;
      msg.data = std::stof(tokens[0]);
      _hdt_publisher.publish(msg);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_driver");
  ros::NodeHandle nh;

  NMEADevice compas(nh);

  while (ros::ok())
    compas.update();

  return 0;
}
