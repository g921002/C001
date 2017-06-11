#ifndef _NMEA_H_
#define _NMEA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minuate;
    uint8_t sec;
		uint8_t quality;
    uint8_t satellites;

    double latitude;
    double longitude;
    double altitude;
    double speed;
    double course;
    double hdop;
}GPS_INFO_t;

void nmea_parse_gprmc(char *, GPS_INFO_t *);
void nmea_parse_gpgga(char *, GPS_INFO_t *);
void nmea_parse_gpvtg(char *, GPS_INFO_t *);
char * strsep(char **stringp, const char *delim);

#endif

