/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vam_util.c
 @brief  : this file include some common function
 @author : wangyifeng
 @history:
           2014-6-24    wangyifeng    Created file
           ...
******************************************************************************/
#include <math.h>
#include <stm32f4xx.h>
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "nmea.h"



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

#define EARTH_RADIUS  6371.004
#define PI 3.1415926
#define RAD(d) ((d)*PI/180.0)

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

#if 0
static double getDistanceVer1(double lat1, double lng1, double lat2, double lng2)

{

   double radLat1 = rad(lat1);

   double radLat2 = rad(lat2);

   double radLng1 = rad(lng1);

   double radLng2 = rad(lng2);

   double s = acos(sin(radLat1)*sin(radLat2)+cos(radLat1)*cos(radLat2)*cos(radLng1-radLng2));

   s = s * EARTH_RADIUS;

   return s;

}
#endif

double getDistanceVer2(double lat1, double lng1, double lat2, double lng2)

{

   double radLat1 = RAD(lat1);

   double radLat2 = RAD(lat2);

   double a = radLat1 - radLat2;

   double b = RAD(lng1) - RAD(lng2);

   double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));

   s = s * EARTH_RADIUS;

   return s;

}

double vsm_get_distance(vam_position_t *p_src, vam_position_t *p_dest)
{
    double d = 1000.0;

    d *= getDistanceVer2(p_src->lat, p_src->lon, p_dest->lat, p_dest->lon);

    #if 0
    {
        char str[64];
        sprintf(str,"%f", d);
        rt_kprintf("distance:%s\n", str);
    }
    #endif

    return d;
}


const char *_directfromangle(int angle)
{
    static const char *dir[] = {
        "北",
        "东北",
        "东",
        "东南",
        "南",
        "西南",
        "西",
        "西北",
    };

    int i;

    if (angle <= 10){
        i = 0;
    }
    else if(angle < 80){
        i = 1;
    }
    else if(angle <= 100){
        i = 2;
    }
    else if(angle < 170){
        i = 3;
    }
    else if(angle <= 190){
        i = 4;
    }
    else if(angle < 260){
        i = 5;
    }
    else if(angle <= 280){
        i = 6;
    }
    else if(angle < 350){
        i = 7;
    }
    else{
        i = 0;
    }

    return dir[i];
}


double vsm_get_relative_pos(vam_stastatus_t *p_src, vam_stastatus_t *p_dest)
{
    double lat1, lng1, lat2, lng2, lat3, lng3;
    double distance_1_2, distance_2_3;
    double angle, delta;

    /* reference point */
    lat1 = p_src->pos.lat;
    lng1 = p_src->pos.lon;

    /* destination point */
    lat2 = p_dest->pos.lat;
    lng2 = p_dest->pos.lon;

    /* temp point */
    lat3 = lat1;
    lng3 = lng2;

    distance_1_2 = getDistanceVer2(lat1, lng1, lat2, lng2);
    distance_2_3 = getDistanceVer2(lat2, lng2, lat3, lng3);
    angle = acos(distance_2_3/distance_1_2)*180/PI;

    /* calculate the relative angle against north, clockwise  */
    if (lat2 >= lat1){
    /* north */
        if (lng2 >= lng1){
        /* easts */
            //equal
        }
        else{
            angle = 360-angle;
        }
    }
    else{
    /* south */
        if (lng2 >= lng1){
        /* easts */
            angle = 180-angle;
        }
        else{
            angle = 180+angle;
        }
    }

    /* calculate the angle detra between local front and remote position  */
    if (angle > p_src->dir){
        delta = angle - p_src->dir;
    }
    else{
        delta = p_src->dir - angle;
    }

    if (delta > 180){
        delta = 360 - delta;
    }

    distance_1_2 *= 1000; /* convert from Km to m */

    rt_kprintf("My head:%s(%d),ID:%d%d%d%d, Your pos:%s(%d),ID:%d%d%d%d, Our delta:%d, distance:%d\n", \
               _directfromangle((int)p_src->dir), (int)p_src->dir,p_src->pid[0],p_src->pid[1],p_src->pid[2],p_src->pid[3],\
               _directfromangle((int)p_dest->dir), (int)p_dest->dir,p_dest->pid[0],p_dest->pid[1],p_dest->pid[2],p_dest->pid[3],\
              (int)delta, (int)distance_1_2);

    return (delta <= 45)? distance_1_2:(-distance_1_2);
}



double vsm_get_relative_dir(vam_stastatus_t *p_src, vam_stastatus_t *p_dest)
{
    double delta;

    /* calculate the angle detra between local front and remote position  */
    if (p_dest->dir > p_src->dir){
        delta = p_dest->dir - p_src->dir;
    }
    else{
        delta = p_src->dir - p_dest->dir;
    }

    if (delta > 180){
        delta = 360 - delta;
    }

    return delta;
}


