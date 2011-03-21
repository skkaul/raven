/*
 *  ROS GPSD Map Viewer using OpenStreetMap & OSMGpsMap
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "gpsd_viewer/gpsd_viewer.h"
#include <ros/package.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <math.h>
#include "gpsd_viewer/cmd.h"
#define pi 3.14
namespace{
ros::Publisher pub_cmd;
gpsd_viewer::cmd c;
double lastlat,lastlon;
int k=0;
int i=0;
int m=0;
double temp1[6];
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) 
{
  return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) 
{
  return (rad * 180 / pi);
}

double distance(double lat1, double lon1)
{

  double theta, dist;
fprintf(stderr, "lastlat %f lastlon %f\n", lastlat, lastlon);


  theta = lon1 - lastlon;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lastlat)) + cos(deg2rad(lat1)) * cos(deg2rad(lastlat)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  dist = dist * (1.609344);
dist=dist*1000;
c.distance[i]=dist;
double dLon = deg2rad(theta);
double y = sin(dLon) * cos(lat1);
double x = cos(lastlat)*sin(lat1) -
        sin(lastlat)*cos(lat1)*cos(dLon);
double brng =atan2(y,x);
c.angle[i]=brng;
fprintf(stderr,"brng %f\n",brng);
lastlat =lat1;
lastlon=lon1;
c.header.frame_id="cmd";
c.header.stamp = ros::Time::now();
c.num_of_waypoints=i+1;
//c.distance[]={0,0,0,0,0};
//c.angle[]={0,0,0,0,0};
//temp1[i]=dist;
//m++;
//if(m==5)
//{
//for(i=0;i=5;i++)
//c.distance[i]=temp1[i];
//}
if(i==5)

{
pub_cmd.publish(c);
}
i++;
for(int j=0;j<=5;j++)
{

fprintf(stderr,"temp %f\n",c.distance[j]);

}
fprintf(stderr,"m %d\n",m);
  return (dist);
}

AppData *data;
static gboolean
on_button_press_event (GtkWidget *widget, GdkEventButton *event, gpointer user_data)
{
    OsmGpsMapPoint coord;
    float lat, lon;
    OsmGpsMap *map = OSM_GPS_MAP(widget);

double dist;
double lat1,lon1;

    if (event->type == GDK_2BUTTON_PRESS) {
        osm_gps_map_convert_screen_to_geographic(map, event->x, event->y, &coord);
        osm_gps_map_point_get_degrees(&coord, &lat, &lon);
	fprintf(stderr, "lat %f lon %f\n",lat,lon);
lat1=(double)lat;
lon1=(double)lon;
//send the function using last latitude,longitude and present latitude and longitude
if(k==0)
{
k=1;
dist=distance(0,0);
fprintf(stderr, "distance betwwn two points %f", dist);
lastlat=lat1;
lastlon=lon1;
}
else
{
dist=distance(lat1,lon1);
fprintf(stderr, "distance betwwn two points %f", dist);
//i=i+1;
}
        if (event->button == 1) {
            osm_gps_map_gps_add (map,lat,lon,g_random_double_range(0,360));
        }
    }

    	
    return FALSE;
}



void gpsFixCallback (const gps_common::GPSFix::ConstPtr & msg)
{
  // **** get GTK thread lock
  gdk_threads_enter ();
  
  gint pixel_x,pixel_y;
  OsmGpsMapPoint * point = osm_gps_map_point_new_degrees(msg->latitude,msg->longitude);
  osm_gps_map_convert_geographic_to_screen(data->map, point, &pixel_x, &pixel_y);
     
  if (OSM_IS_GPS_MAP (data->map)){

		// **** Center map on gps data received
		if(data->lock_view)
		{
			update_uav_pose_osd(data->osd,TRUE, pixel_x, pixel_y);
			osm_gps_map_set_center (data->map, msg->latitude, msg->longitude);
		}
		else
		{
			update_uav_pose_osd(data->osd,FALSE, pixel_x, pixel_y);
			osm_gps_map_gps_clear(data->map);
		}
	
		// **** Add point to the track
		osm_gps_map_track_add_point (data->current_track, point);
  }
	
  // **** release GTK thread lock 
  gdk_threads_leave ();
}

/**
 * @fn void *startROS (void *user)
 * @brief ROS thread.
 * 
 * The main program wait until "ros_param_read" in order to allow the <br>
 * ROS params to be also the Gtk Window and Widgets params.
 * Then the ROS thread wait to the widgets creation before subcribing<br>
 * to any topics, avoid to call public widget function for a widget not<br>
 * yet created.
 */
void *startROS (void *user)
{
  if (user != NULL)
  {
    struct arg *p_arg = (arg *) user;

    ros::init (p_arg->argc, p_arg->argv, "gpsd_viewer");
    ros::NodeHandle n;

    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    ROS_INFO ("Starting GPSD Viewer");

	 // -----------------------------------------------------------------      
    // **** allow widget creation
    data->ros_param_read = true;

    // **** wait to widget creation
    while (!data->widget_created)
    {
      ROS_DEBUG ("Waiting widgets creation");
    }

    // -----------------------------------------------------------------      
    // **** topics subscribing
	 ros::Subscriber fix_sub;
    fix_sub = n.subscribe ("/fix", 1, &gpsFixCallback);
    pub_cmd= n.advertise<gpsd_viewer::cmd>("cmd",5);
    ROS_INFO ("Spinning");
    ros::spin ();
  }

  // **** stop the gtk main loop
  if (GTK_IS_WINDOW (data->window))
  {
    gtk_main_quit ();
  }
  pthread_exit (NULL);
}

/**
 * @fn int main (int argc, char **argv)
 * @brief Main program & Gtk thread.
 * 
 * Create window and all widgets, then set there parameters to be the <br>
 * ROS params.
 */
int main (int argc, char **argv)
{
  GtkBuilder *builder;
  GError *error = NULL;
  char gui_filename[FILENAME_MAX];
  int start_zoom = 15;
  OsmGpsMapPoint ccny_coord = { 33.421355, -111.928253 };
OsmGpsMapTrack *rightclicktrack;

  struct arg param;
  param.argc = argc;
  param.argv = argv;

  pthread_t rosThread;

  // **** init threads 
  g_thread_init (NULL);
  gdk_threads_init ();
  gdk_threads_enter ();

  // **** init gtk 
  gtk_init (&argc, &argv);

  // **** get the glade gui file
  std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
  sprintf (gui_filename, "%s/%s", package_path.c_str (), "gui.glade");

  // Allocate data structure
  data = g_slice_new (AppData);

  // Create new GtkBuilder object
  builder = gtk_builder_new ();
  // Load UI from file
  if (!gtk_builder_add_from_file (builder, gui_filename, &error))
  {
    g_warning ("%s", error->message);
    g_free (error);
    pthread_exit (NULL);
  }

  // Get main window pointer from UI
  data->window = GTK_WIDGET (gtk_builder_get_object (builder, "window"));

  // **** create ROS thread
  pthread_create (&rosThread, NULL, startROS, &param);

  // **** wait ros finish read params
  while (!data->ros_param_read)
  {
    ROS_DEBUG ("Waiting ROS params");
  }

  // Some initialisation
  gdk_window_set_debug_updates (false);
  data->draw_path = false;
  data->map_provider = OSM_GPS_MAP_SOURCE_GOOGLE_SATELLITE;
  data->map_zoom_max = 18;
  data->map_current_zoom = start_zoom;
  data->repo_uri = osm_gps_map_source_get_repo_uri (data->map_provider);
  data->friendly_name = osm_gps_map_source_get_friendly_name (data->map_provider);

  char *mapcachedir;
  mapcachedir = osm_gps_map_get_default_cache_directory ();
  data->cachedir = g_build_filename (mapcachedir, data->friendly_name, NULL);
  g_free (mapcachedir);

  // Create the OsmGpsMap object
  data->map = (OsmGpsMap *) g_object_new (OSM_TYPE_GPS_MAP, 
								"map-source", data->map_provider,
                        "tile-cache", data->cachedir, 
                        "proxy-uri", g_getenv ("http_proxy"),
                        "auto-center",FALSE, NULL);

  //Set the starting coordinates and zoom level for the map
  osm_gps_map_set_zoom (data->map, start_zoom);
  osm_gps_map_set_center (data->map, ccny_coord.rlat, ccny_coord.rlon);

  data->osd = gpsd_viewer_osd_new();
  g_object_set(GPSD_VIEWER_OSD(data->osd),
								"show-scale",true,
                        "show-coordinates",true,
                        "show-dpad",true,
                        "show-zoom",true,
                        "show-gps-in-dpad",true,
                        "show-gps-in-zoom",false,
                        "dpad-radius", 30,
                        NULL);
                                                                     
  osm_gps_map_layer_add(OSM_GPS_MAP(data->map), OSM_GPS_MAP_LAYER(data->osd));
    
  data->current_track = osm_gps_map_track_new();
    //Add a second track for right clicks
    rightclicktrack = osm_gps_map_track_new();
    osm_gps_map_track_add(OSM_GPS_MAP(data->map), rightclicktrack);
  // Add the map to the box
  data->map_box = GTK_WIDGET (gtk_builder_get_object (builder, "hbox2"));
  gtk_box_pack_start (GTK_BOX (data->map_box), GTK_WIDGET (data->map), TRUE, TRUE, 0);

  data->map_container = GTK_WIDGET (gtk_builder_get_object (builder, "hbox1"));

  // Connect signals
g_signal_connect (G_OBJECT (data->map), "button-press-event",
                G_CALLBACK (on_button_press_event), (gpointer) rightclicktrack);
  gtk_builder_connect_signals (builder, data);

  // Destroy builder, since we don't need it anymore
  g_object_unref (G_OBJECT (builder));

  // Show window. All other widgets are automatically shown by GtkBuilder
  gtk_widget_show_all (data->window);

  // **** allow ROS spinning
  data->widget_created = true;
  
  // Start main loop
  gtk_main ();
  gdk_threads_leave ();
  return 0;
}
