#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define DISPLAY_STRING
int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int serial_write(int fd, char *cmd)
{
	int wlen = 0;
    wlen = write(fd, cmd, strlen(cmd));
    if (wlen != strlen(cmd)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */

	printf("write [%s] %lu okay!\n", cmd, strlen(cmd));

    return wlen;
}

static int serial_read(int fd, char *buf, int buflen)
{
    int rdlen, len;
    printf("Start read ...\n");
    rdlen = read(fd, buf, buflen - 1);
    len = rdlen;
    if (rdlen > 0) {
#ifdef DISPLAY_STRING
        buf[rdlen] = 0;
        printf("Read %d: [%s]\n", rdlen, buf);
#else /* display hex */
        unsigned char   *p;
        printf("Read %d:", rdlen);
        for (p = buf; rdlen-- > 0; p++)
            printf(" 0x%x", *p);
            printf("\n");
#endif
    } else if (rdlen < 0) {
        printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    }

    if (len < 4) {
         printf("read error !\n");
         return -1;
     }

     // TODO search "OK\r\n"
     if (!strstr(buf, "OK\r\n")) {
        printf("modem error!\n");
         return -1;
     }

    return len;
}


void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


int main(int argc, char **argv)
{
    char command[64];
    char *apnname[3] = {"CMNET", "CTNET", "3GNET"};
    char portname[16] = "/dev/ttyUSB1";
    int fd = -1;
    int wlen, rlen;
    char buf[256];
    int port_index;
    int apn_index;

    if (argc != 3) {
        printf("4G connector <ttyUSBx> <APN>\n");
        printf("x = 0, 1, 2, ...\n");
        printf("APN=0: cmnet\n");
        printf("APN=1: ctnet\n");
        printf("APN=2: 3gnet\n");
        printf("Using default ttyUSB1 3gnet\n");
        port_index = 1;
        apn_index = 2;
    } else {
        port_index = strtol(argv[1], NULL, 10);
        if (errno) {
            printf("arg error, use the default\n");
            port_index = 1;
        }

        if (port_index < 0 || port_index > 6) {
            printf("arg out of range, use the default\n");
            port_index = 1;
        }
        
        apn_index = strtol(argv[2], NULL, 10);
        if (errno) {
            printf("arg error, use the default\n");
            apn_index = 2;
        }

        if (apn_index < 0 || apn_index > 2) {
            printf("arg out of range, use the default\n");
            apn_index = 1;
        }        
    }

    snprintf(portname, sizeof(portname), "/dev/ttyUSB%x", port_index);

    snprintf(command, sizeof(command), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", apnname[apn_index]);

	printf("Open %s Using [%s]\n", portname, command);
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */

    serial_write(fd, "ati\r\n");
    rlen = serial_read(fd, buf, sizeof(buf));
    if (rlen < 0) {
        printf("read error !\n");
        goto ERROR;
    }

    sleep(1);

    //serial_write(fd, "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");
    //serial_write(fd, "AT+CGDCONT=1,\"IP\",\"CTNET\"\r\n");
    serial_write(fd, command);
    rlen = serial_read(fd, buf, sizeof(buf));
    if (rlen < 0) {
        printf("read error !\n");
        goto ERROR;
    }

    sleep(1);

    serial_write(fd, "AT+ZECMCALL=1\r\n");
    rlen = serial_read(fd, buf, sizeof(buf));
    if (rlen < 0) {
        printf("read error !\n");
        goto ERROR;
    }

ERROR:
    close(fd);
}
