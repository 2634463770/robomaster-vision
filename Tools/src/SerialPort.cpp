/*
 * 頂頂頂頂頂頂頂頂頂　頂頂頂頂頂頂頂頂頂
 * 頂頂頂頂頂頂頂　　　　　頂頂
 * 　　　頂頂　　　頂頂頂頂頂頂頂頂頂頂頂
 * 　　　頂頂　　　頂頂頂頂頂頂頂頂頂頂頂
 * 　　　頂頂　　　頂頂　　　　　　　頂頂
 * 　　　頂頂　　　頂頂　　頂頂頂　　頂頂
 * 　　　頂頂　　　頂頂　　頂頂頂　　頂頂
 * 　　　頂頂　　　頂頂　　頂頂頂　　頂頂
 * 　　　頂頂　　　頂頂　　頂頂頂　　頂頂
 * 　　　頂頂　　　　　　　頂頂頂
 * 　　　頂頂　　　　　　頂頂　頂頂　頂頂
 * 　頂頂頂頂　　　頂頂頂頂頂　頂頂頂頂頂
 * 　頂頂頂頂　　　頂頂頂頂　　　頂頂頂頂
 */

#include <unistd.h>  // UNIX Standard Definitions
#include <fcntl.h>   // File Control Definitions
#include <errno.h>   // ERROR Number Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include "SerialPort.h"

Data_Save ds;

int watch_dog;

bool bool_serial;

SerialPort::SerialPort(const char *filename)
{
    file_name_ = filename;
    buadrate_ = B115200;
    success_ = false;
    //    serial_mode = NO_INIT;
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port                                           // No terminal will control the process
    last_fd = fd;
    if (fd == -1)
    {
#ifdef DEBUG_SERIAL
        printf("open_port wait to open %s .\n", file_name_);
#endif
        //        NOTICE("wait serial " << file_name_,1);
        return;
    }
    else if (fd != -1)
    {
        fcntl(fd, F_SETFL, 0);
        // fcntl(fd, F_SETFL, FNDELAY); // no late

#ifdef DEBUG_SERIAL
        printf("port is open %s.\n", file_name_);
#endif
    }
    struct termios port_settings;         // structure to store the port settings in
    cfsetispeed(&port_settings, B115200); // set baud rates

    cfsetospeed(&port_settings, B115200);
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK; // disable break processing
    port_settings.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;

    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

    //    char write_buffer[]="hello word";
    //    int bytes_written=0;
    //    bytes_written=write(fd,write_buffer,sizeof(write_buffer));
}

int open_name = 0;
void SerialPort::restart_serial(void)
{
    //    cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    if (open_name > 10)
    {
        open_name = 0;
    }
    if (open_name % 2 == 1)
    {
        fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
    }
    else
    {
        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
    }
    //    cout << serial_mode << endl;
    if (fd == -1 && last_fd != -1)
    {
#ifdef DEBUG_SERIAL
        printf("open_port wait to open %s .\n", file_name_);
#endif
        //        NOTICE("wait serial",1);
        last_fd = fd;
        return;
    }
    else if (fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL, 0);
        // fcntl(fd, F_SETFL, FNDELAY); // no late

        //        NOTICE("port is open",1);
#ifdef DEBUG_SERIAL
        printf("port is open %s.\n", file_name_);
#endif
        last_fd = fd;
    }
    else
    {
        last_fd = fd;
        return;
    }
    struct termios port_settings; // structure to store the port settings in
    if (buadrate_ == 0)
    {
        cfsetispeed(&port_settings, B115200); // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if (buadrate_ == 1)
    {
        cfsetispeed(&port_settings, B115200); // set baud rates
        cfsetospeed(&port_settings, B115200);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK; // disable break processing
    port_settings.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port
}

// read the datas,which is enermy color, aim mode and camera vertical angle
bool SerialPort::read_data()
{
    tcflush(fd, TCIFLUSH);                /* Discards old data in the rx buffer            */
    unsigned char read_buffer[READ_SIZE]; /* Buffer to store the data received              */
    long bytes_read = 0;                  /* Number of bytes read by the read() system call */

    bytes_read = read(fd, &read_buffer, READ_SIZE); /* Read the data                   */

    // ds.plus = 0;
    // ds.aim_open = 1;
    // ds.mode_num = 0;
    // ds.car_color = 0;
    // ds.car_angle = 0;   //底盘的角度差
    // ds.car_speed_x = 0; //底盘的运动速度
    // ds.car_speed_y = 0;
    // ds.bullet_velocity = 20;
    // ds.camera_yaw_angle = 0;
    // ds.camera_pit_angle = 0;

#ifdef DEBUG_SERIAL
    cout << "bytes_read : " << bytes_read << endl;
#endif

    if (bytes_read == -1 || bytes_read == 0)
    { 

// #ifdef DEBUG_SERIAL
        // cout << "can not read!"
            //  << "  " << bytes_read << endl;
// #endif
        restart_serial();

        // serial_watch = 0;

        return 0;
    }
    else
    {

#ifdef DEBUG_SERIAL
        cout << "Nice fuck the read!"
             << "   " << bytes_read << endl;
#endif

        if (read_buffer[0] == 0xAA && read_buffer[1] == 0xBB && read_buffer[READ_SIZE-1] == 0xCC)
        {
            // 0 红蓝方； 1 装甲板； 2 yaw正负； 3 pit正负； 4 移动正负； 5 是否在自瞄
            bool plot[8];

            plot[0] = static_cast<bool>((read_buffer[2] >> 0) & 0x01);
            plot[1] = static_cast<bool>((read_buffer[2] >> 1) & 0x01);
            plot[2] = static_cast<bool>((read_buffer[2] >> 2) & 0x01);
            plot[3] = static_cast<bool>((read_buffer[2] >> 3) & 0x01);
            plot[4] = static_cast<bool>((read_buffer[2] >> 4) & 0x01);
            plot[5] = static_cast<bool>((read_buffer[2] >> 5) & 0x01);
            plot[6] = static_cast<bool>((read_buffer[2] >> 6) & 0x01);
            plot[7] = static_cast<bool>((read_buffer[2] >> 7) & 0x01);

            ds.car_color = plot[0];   // red and blue   (1 为红色, 0 为蓝色)
            ds.mode_num = plot[1];    // 装甲板， 能量机关（0 为装甲板，1 为能量机关）
            bool yaw_plot = plot[2];  // 云台yaw 正负    （0 为正， 1为负）
            bool pit_plot = plot[3];  // 云台pitch正负   （0 为正， 1为负）
            bool dist_plot = plot[4]; // 车身x轴移动正负  （0 为正， 1为负）
            ds.aim_open = plot[5];    // 是否开启自瞄     （0 为关闭， 1为开启）

            int32_t yaw_ = read_buffer[3] << 24 | read_buffer[4] << 16 | read_buffer[5] << 8 | read_buffer[6];  // 云台yaw
            int32_t pit_ = read_buffer[7] << 24 | read_buffer[8] << 16 | read_buffer[9] << 8 | read_buffer[10]; // 云台pitch
            ds.bullet_velocity = static_cast<double>(read_buffer[11]) / 8;                                      // 子弹速度
            if(ds.bullet_velocity<13)
                ds.bullet_velocity = 20;
            ds.car_speed_x = static_cast<double>(read_buffer[12] << 8 | read_buffer[13]) / 10000;               // 车身x轴移动
            ds.car_id = static_cast<int>(read_buffer[14]);                                                      // 兵种号码

#ifdef INFANTRY
            ds.DetectionMode = static_cast<int>(read_buffer[15]);
            ds.PitchCompensation = static_cast<int>(read_buffer[16]) - 100;
            ds.YawCompensation = static_cast<int>(read_buffer[17]) - 100;
            ds.PredictionCompensation = static_cast<int>(read_buffer[18]) - 100;
            ds.Auto_fire = static_cast<int>(read_buffer[19]);//
#endif

            // Bullet_Velocity_Control();

            if (yaw_plot)
                ds.camera_yaw_angle = static_cast<double>(yaw_ * -1) / 10000;
            else
                ds.camera_yaw_angle = static_cast<double>(yaw_ * 1) / 10000;

            if (pit_plot)
                ds.camera_pit_angle = static_cast<double>(pit_ * -1) / 10000;
            else
                ds.camera_pit_angle = static_cast<double>(pit_ * 1) / 10000;

            if (dist_plot)
                ds.plus = -1;
            else
                ds.plus = 1;

            // cout << "plot : ";
            // for (int x = 0; x < 8; x++)
            // {
            //     cout << plot[x] << " ";
            // }
            // cout << endl;
            // cout << ds.camera_yaw_angle << endl;
            // cout << ds.camera_pit_angle << endl;
            // cout << ds.bullet_velocity << endl;
            // cout << ds.car_speed_x << endl;
            // cout << ds.car_id << endl;

            return 1;
        }
        else
        {

            cout << "this is not the data I want ! " << endl;

            return 0;
        }
    }

    return 0;
    // 0xaa -> 170
    // 0xbb -> 187
}

void SerialPort::send_data(const struct Data_Get &data)
{
    if (data.size != write(fd, data.raw_data, data.size))
    {
        open_name++;

        cout << "!!! send data failure !!!" << fd << endl;
#ifdef DEBUG_SERIAL
#endif

#ifdef DEBUG_SERIAL 
        cout << "restart fd" << fd << endl;
#endif
        restart_serial();

        watch_dog = 0;
        return;
    }
    else
    {
#ifdef DEBUG_SERIAL
        cout << "!!! send data success !!!" << fd << endl;
#endif
        return;
    }
}

void Data_Get::get_xy_data(int32_t x, int32_t y, uint8_t found, uint8_t fire)
{
    // cout << " yaw : " << x << "    pit : " << y << "    found : " << static_cast<int>(found) << "   fire : " << static_cast<int>(fire) << endl;

    size = 13;

    raw_data[ 0] = 0xAA;
    raw_data[ 1] = 0xAA;
    raw_data[ 2] = (x >> 24) & 0xff;
    raw_data[ 3] = (x >> 16) & 0xff;
    raw_data[ 4] = (x >>  8) & 0xff;
    raw_data[ 5] = (x >>  0) & 0xff;
    raw_data[ 6] = (y >> 24) & 0xff;
    raw_data[ 7] = (y >> 16) & 0xff;
    raw_data[ 8] = (y >>  8) & 0xff;
    raw_data[ 9] = (y >>  0) & 0xff;
    raw_data[10] = found;
    raw_data[11] = fire; //是否开火
    // raw_data[12] = 0; //是否开火

    raw_data[12] = 0xBB;
    
    return;
}
