

#include "nmea.h"
void nmea_parse_gprmc(char *nmea, GPS_INFO_t *loc)
{
    int Column = 0;
    double ftmp = 0.0;
    double fd = 0.0;
    double fm = 0.0;
    char *delim = ",*";
    char *token;
    //char *s = strdup(nmea);
    char ss[100];
    char *s = ss;
    strcpy(s, nmea);

    for(token = strsep(&s, delim); token != NULL ; token = strsep(&s, delim))
    {
        if(strcmp(token, "") != 0)
        {
            switch(Column)
            {

            case 0:
                break;

            case 1: //utc time
                ftmp = atof(token);
                loc->hour = (int)ftmp / 10000 ;
                loc->minuate = ((int)ftmp / 100) % 100;
                loc->sec = (int)ftmp % 100;
                break;

            case 2: //status : V-Navigation receiver warning  A-Data Valid
                //loc->quality = atoi(token);
                break;

            case 3: //latitude
                ftmp = atof(token);
                fm = modf(ftmp / 100.0, &fd) * 5.0 / 3.0;
                loc->latitude = fd + fm;
                break;

            case 4:
                if(token[0] == 'S')
                {
                    loc->latitude *= -1;
                }

                break;

            case 5: //longitude
                ftmp = atof(token);
                fm = modf(ftmp / 100.0, &fd) * 5.0 / 3.0;
                loc->longitude = fd + fm;
                break;

            case 6:
                if(token[0] == 'W')
                {
                    loc->latitude *= -1;
                }

                break;

            case 7: //Speed over ground
                loc->speed = atoi(token);
                break;

            case 8: //Course over ground
                loc->course = atof(token);
                break;

            default:

                break;
            }
        }

        Column++;
    }
}


void nmea_parse_gpgga(char *nmea, GPS_INFO_t *loc)
{
    int Column = 0;
    double ftmp = 0.0;
    double fd = 0.0;
    double fm = 0.0;
    char *delim = ",*";
    char *token;
    //char *s = strdup(nmea);
    char ss[100];
    char *s = ss;
    strcpy(s, nmea);

    for(token = strsep(&s, delim); token != NULL ; token = strsep(&s, delim))
    {
        if(strcmp(token, "") != 0)
        {
            switch(Column)
            {

            case 0:
                break;

            case 1: //utc time
                ftmp = atof(token);
                loc->hour = (int)ftmp / 10000 ;
                loc->minuate = ((int)ftmp / 100) % 100;
                loc->sec = (int)ftmp % 100;
                break;

            case 2: //latitude
                ftmp = atof(token);
                fm = modf(ftmp / 100.0, &fd) * 5.0 / 3.0;
                loc->latitude = fd + fm;
                break;

            case 3:
                if(token[0] == 'S')
                {
                    loc->latitude *= -1;
                }

                break;

            case 4: //longitude
                ftmp = atof(token);
                fm = modf(ftmp / 100.0, &fd) * 5.0 / 3.0;
                loc->longitude = fd + fm;
                break;

            case 5:
                if(token[0] == 'W')
                {
                    loc->latitude *= -1;
                }

                break;

            case 6: //fix quality
                loc->quality = atoi(token);
                break;

            case 7: //Number of Satellites
                loc->satellites = atoi(token);
                break;

            case 8: //HDOP
                loc->hdop = atof(token);
                break;

            case 9: //Altitude
                loc->altitude = atof(token);
                break;

            default:

                break;
            }
        }

        Column++;
    }
}

void nmea_parse_gpvtg(char *nmea, GPS_INFO_t *loc)
{
    int Column = 0;
    char *delim = ",*";
    char *token;
    //char *s = strdup(nmea);
    char ss[100];
    char *s = ss;
    strcpy(s, nmea);

    for(token = strsep(&s, delim); token != NULL ; token = strsep(&s, delim))
    {
        if(strcmp(token, "") != 0)
        {
            switch(Column)
            {

            case 0:
                break;

            case 1: //degrees true
                //loc->course = atof(token);
                break;

            case 2: //skip
                break;

            case 3: //degrees magnetic,skip

                break;

            case 4: //skip

                break;

            case 5: //Speed, in knots
                //1 knots =1.85200 km/hr
                //1 knots =0.51444 m/s
                loc->speed = atof(token) * 1.852;
                break;

            case 6: //skip
                break;

            case 7: //Speed over ground in kilometers/hour (kph),skip
                break;

            default:
                break;
            }
        }

        Column++;
    }
}


char * strsep(char **stringp, const char *delim)
{
    char *s;
    const char *spanp;
    int c, sc;
    char *tok;

    if((s = *stringp) == NULL)
        return (NULL);

    for(tok = s;;)
    {
        c = *s++;
        spanp = delim;

        do
        {
            if((sc = *spanp++) == c)
            {
                if(c == 0)
                    s = NULL;
                else
                    s[-1] = 0;

                *stringp = s;
                return (tok);
            }
        }
        while(sc != 0);
    }

    /* NOTREACHED */
}
