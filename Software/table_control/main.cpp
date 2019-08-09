#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/input.h>
#include <fcntl.h>

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <iostream>
#include <thread>
#include <mutex>

#include <boost/program_options.hpp>
#include <boost/asio.hpp>

class PlaneListener
{
public:
    PlaneListener();
};

PlaneListener::PlaneListener()
{
}

using namespace std;
namespace po = boost::program_options;
namespace asio = boost::asio;

// 700 - 3300

struct Point
{
    Point(double x = 0, double y = 0) 
    { this->x = x; this->y = y; }

    double x, y;

    Point operator-(const Point &b) const
    {
        return Point(x-b.x, y-b.y);
    }

    Point operator+(const Point &b) const
    {
        return Point(x+b.x, y+b.y);
    }

    Point operator*(const float scal) const
    {
        return Point(x*scal, y*scal);
    }
};

ostream& operator<<(ostream& os, const Point& pnt)
{
    os << "(" << pnt.x << ", " << pnt.y << ")";
    return os;
}


class ControlSystem
{
public:
    ControlSystem();
    ~ControlSystem();

    void updateBallPosition(Point &pos);
    void resetBall();
    void placeBall();

    static Point adc2normalPoint(int32_t x, int32_t y);

private:
    asio::io_service io;
    asio::serial_port m_port;

    void threadRoutine();
    bool m_isActive;
    bool m_isPlaced;

    Point m_pos;

    mutex m_io_mutex;
    thread _thread;

    static constexpr double min_x = 700;
    static constexpr double min_y = 700;
    static constexpr double max_x = 3300;
    static constexpr double max_y = 3300;
};

ControlSystem::ControlSystem() : 
    m_isActive(true), m_isPlaced(false),
    m_port(io, "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_404-if00")
{
    _thread = thread(&ControlSystem::threadRoutine, this);

    cout << "Thread created" << endl;

    if ( !m_port.is_open() )
        throw logic_error("Failed to open serial port");
}

ControlSystem::~ControlSystem()
{
    m_isActive = false;
}

void ControlSystem::resetBall()
{
    lock_guard<std::mutex> lock(m_io_mutex);

    m_isPlaced = false;
    uint8_t buffer[3] = {167, 253};
    asio::write(m_port, asio::buffer("$", 1));
    asio::write(m_port, asio::buffer(buffer, sizeof(buffer)));
}

void ControlSystem::placeBall()
{
    lock_guard<std::mutex> lock(m_io_mutex);
    
    m_isPlaced = true;
    uint8_t buffer[3] = {165, 252};
    asio::write(m_port, asio::buffer("%", 1));
    asio::write(m_port, asio::buffer(buffer, sizeof(buffer)));
}

Point ControlSystem::adc2normalPoint(int32_t x, int32_t y)
{
    if ( x < min_x || x > max_x )
        throw out_of_range( "X has invalid value" );

    if ( y < min_y || y > max_y )
        throw out_of_range( "Y has invalid value" );

    double n_x = ((double)x - min_x) / (max_x - min_x);
    n_x = n_x * 2 - 1.;

    double n_y = ((double)y - min_y) / (max_y - min_y);
    n_y = n_y * 2 - 1.;

    return Point(n_x, n_y);
}


void ControlSystem::updateBallPosition(Point &pos)
{
    if ( !m_isPlaced )
        throw logic_error( "Ball is not placed" );

    m_pos = pos;

    // cout << m_pos << endl;
}

void ControlSystem::threadRoutine()
{
    const chrono::milliseconds intervalMillis(10);

    chrono::system_clock::time_point currentTime;
    chrono::system_clock::time_point nextPeriodTime;

    while (m_isActive)
    {
        currentTime = chrono::system_clock::now();
        nextPeriodTime = currentTime + intervalMillis;
        
        m_io_mutex.lock();

        static Point integr_sum;

        if ( m_isPlaced )
        {
            Point reference;
            Point error = reference - m_pos;

            static Point prev_error;

            const double p_rate = 20;
            const double d_rate = 400;
            const double i_rate = 0.000;

            integr_sum = integr_sum + error * i_rate;

            Point incline = error * p_rate + (error - prev_error) * d_rate + integr_sum;
            prev_error = error;

            /* Minus - because of mechanics */
            float buffer[2] = { -incline.x, -incline.y };

            asio::write(m_port, asio::buffer("#", 1));
            asio::write(m_port, asio::buffer(buffer, sizeof(buffer)));

            // cout << "Send: " << buffer[0] << " / " << buffer[1]
                    // << " / Pos: " << m_pos << endl;
        }
        else
        {
            integr_sum = Point();
        }

        m_io_mutex.unlock();

        this_thread::sleep_until(nextPeriodTime);
    }
}

int main(int argc, char *argv[])
{
    string dev;

    ControlSystem system;

    po::options_description desc{"Options"};
    auto opts_init = desc.add_options();
    opts_init("help,h", "Help screen");
    opts_init("device,d", po::value<string>(), "Device to read");

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
        cout << desc << endl;
    else if (vm.count("device"))
        dev = vm["device"].as<string>();

    int fd;
    struct input_event ie;

    cout << "Reading device: " << dev << endl;

    if ((fd = open(dev.c_str(), O_RDONLY)) == -1)
    {
        cerr << "Failed to open device" << endl;
        exit(EXIT_FAILURE);
    }

    int32_t last_pos_x = 0;
    int32_t last_pos_y = 0;

    while (read(fd, &ie, sizeof(struct input_event)))
    {
        switch (ie.type)
        {
        case EV_KEY:
            cout << "Key pressed: " << ie.value << endl;
            if ( ie.value )
                system.placeBall();
            else
                system.resetBall();

            break;

        case EV_ABS:
            if (ie.code == ABS_X)
            {
                last_pos_x = ie.value;
            }
            if (ie.code == ABS_Y)
            {
                last_pos_y = ie.value;
            }

            try
            {
                Point ballPos = ControlSystem::adc2normalPoint( last_pos_x, last_pos_y );
                system.updateBallPosition( ballPos );
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }

            // cout << "New ABS position: ("
            //      << last_pos_x << ", "
            //      << last_pos_y << ")" << endl;
            break;
        }
    }

    close(fd);

    return EXIT_SUCCESS;
}
