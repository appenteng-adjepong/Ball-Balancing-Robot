/*
 * force_sensor.c
 *
 * Copyright 2015  <ubuntu@udoobuntu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

// write your includes here
#include "feather_serial.h"

int serialport_init(const char *serialport, int baud)
{
    struct termios toptions;
    int fd;

    fd = open(serialport, O_RDWR | O_NONBLOCK);

    if (fd == -1)
    {
        return -1;
    }

    if (tcgetattr(fd, &toptions) < 0)
    {
        return -1;
    }

    speed_t brate = baud;
    switch (baud)
    {
    case 4800:
        brate = B4800;
        break;
    case 9600:
        brate = B9600;
        break;
#ifdef B14400
    case 14400:
        brate = B14400;
        break;
#endif
    case 19200:
        brate = B19200;
        break;
#ifdef B28800
    case 28800:
        brate = B28800;
        break;
#endif
    case 38400:
        brate = B38400;
        break;
    case 57600:
        brate = B57600;
        break;
    case 115200:
        brate = B115200;
        break;
    }

    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;                  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);         // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST;                          // make raw
    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &toptions);
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
    {
        return -1;
    }

    return fd;
}

int serialport_close(int fd)
{
    return close(fd);
}

int serialport_writebyte(int fd, const char *str)
{
    int len = 1;
    int n = write(fd, str, len);
    if (n != len)
    {
        return -1;
    }

    return 0;
}

int serialport_write(int fd, const char *str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if (n != len)
    {
        return -1;
    }
    return 0;
}

int serialport_read(int fd, char *buf, int buf_max, int timeout)
{
    char b[1]; // read expects an array, so we give it a 1-byte array
    int i = 0;
    int n = 0;

    do
    {
        n = read(fd, b, 1); // read a char at a time
        if (n == -1)
            return -1; // couldn't read

        if (n == 0)
        {
            usleep(1 * 1000); // wait 1 msec try again
            timeout--;
            continue;
        }

        buf[i] = b[0];
        i++;

    } while (i < buf_max && timeout > 0);

    // buf[i] = 0;  // null terminate the string
    if (strlen(buf) > 0)
    {
        n = 1;
    }

    return n;
}

int serialport_read_until(int fd, char *buf, int buf_max, char eol, int timeout)
{
    char b[1]; // read expects an array, so we give it a 1-byte array
    int i = 0;
    int n = 0;

    do
    {
        n = read(fd, b, 1); // read a char at a time
        if (n == -1)
        {
            return -1; // couldn't read
        }

        if (n == 0)
        {
            usleep(1 * 100); // wait 0.1 msec try again
            timeout--;
            continue;
        }

        buf[i] = b[0];
        i++;

    } while (i < buf_max && timeout > 0 && b[0] != eol);

    // buf[i] = 0;  // null terminate the string
    if (strlen(buf) > 0)
    {
        n = 1;
    }

    return n;
}