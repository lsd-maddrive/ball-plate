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

class PlaneListener
{
public:
    PlaneListener();
};

PlaneListener::PlaneListener()
{
}

using namespace std;

int main(int argc, char *argv[])
{
    int fd;
    const char *dev;
    struct input_event ie;

    if (argc != 2)
    {
        cerr << "Invalid arguments" << endl;
    }

    dev = argv[1];

    if ((fd = open(dev, O_RDONLY)) == -1)
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
        }
    }

    close(fd);

    return EXIT_SUCCESS;
}
