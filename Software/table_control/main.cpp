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

#include <boost/program_options.hpp>

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

int main(int argc, char *argv[])
{
    string dev;

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

            cout << "New ABS position: ("
                 << last_pos_x << ", "
                 << last_pos_y << ")" << endl;
            break;
        }
    }

    close(fd);

    return EXIT_SUCCESS;
}
