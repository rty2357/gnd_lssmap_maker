/**
 * @file gnd_lssmap_maker/src/main.cpp
 *
 * @brief Laser Scan Statistics MAP maker
 **/

#include "gnd/gnd-multi-platform.h"

#include "gnd/gnd_lssmap_maker.hpp"
#include "gnd/gnd_lssmap_maker_config.hpp"

#include "ros/ros.h"
#include "ros/Time.h"
#include "ros/Rate.h"

#include "sensor_msgs/PointCloud.h"
#include "gnd_geometry2d_msgs/msg_pose2d_stamped.h"
#include "gnd/gnd_rosmsg_reader.hpp"

#include <stdio.h>
#include <float.h>

#include "gnd/gnd-matrix-base.hpp"
#include "gnd/gnd-vector-base.hpp"
#include "gnd/gnd-matrix-coordinate.hpp"
#include "gnd/gnd-lssmap-base.hpp"

typedef gnd::lssmap_maker::node_config							node_config_t;

typedef sensor_msgs::PointCloud									msg_pointcloud_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_pointcloud_t>	msgreader_pointcloud_t;
typedef gnd_geometry2d_msgs::msg_pose2d_stamped					msg_pose_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_pose_t>		msgreader_pose_t;

typedef gnd::lssmap::cmap_t										cmap_t;
typedef gnd::lssmap::lssmap_t									lssmap_t;

int main(int argc, char **argv) {
	node_config_t			node_config;

	{ // ---> start up, read configuration file
		if( argc > 1 ) {
			if( gnd::lssmap_maker::fread_node_config( argv[1], &node_config ) < 0 ) {
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read config file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( gnd::lssmap_maker::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}
	} // <--- start up, read configuration file


	{ // ---> initialize rosÅ@platform
		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
		fprintf(stdout, " node: \"%s\"\n", node_config.node_name.value);
	} // <--- initialize rosÅ@platform



	// ---> variables
	ros::NodeHandle			nh_ros;					// ros nodehandle

	ros::Subscriber			subsc_pointcloud;		// point-cloud subscriber
	msgreader_pointcloud_t	msgreader_pointcloud;	// point-cloud message reader and storage
	msg_pointcloud_t		msg_pointcloud;			// operating point-cloud

	ros::Subscriber			subsc_pose;				// pose subscriber
	msg_pose_t				msg_pose;				// pose message reader and storage
	msgreader_pose_t		msgreader_pose;			// operating pose

	cmap_t					lssmap_counting;		// counting map of laser scan statistics

	FILE* fp_txtlog = 0;								// debug file stream
	// <--- variables



	{ // ---> initialize node
		int phase = 0;
		ros::Time time_start = ros::Time::now();
		fprintf(stdout, "---------- initialize ----------\n");

		// ---> show initialize phase task
		if( ros::ok() ) {
			fprintf(stdout, " initialization task\n");
			fprintf(stdout, "   %d. initialize global pose topic subscriber\n", ++phase);
			fprintf(stdout, "   %d. initialize point-cloud topic subscriber\n", ++phase);
			fprintf(stdout, "   %d. initialize map for laser scan data counting\n", ++phase);
			if ( node_config.text_log.value[0] ) {
				fprintf(stdout, "   %d. create log file\n", ++phase);
			}
			fprintf(stdout, "\n");
		} // <--- show initialize phase task


		// ---> initialize robot pose subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => initialize robot pose topic subscriber\n");

			if( !node_config.topic_name_pose.value[0] ) {
				fprintf(stderr, "    ... error: laser scan topic name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_pose.item);
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pose.value);

				// allocate buffer
				msgreader_pose.allocate(1000);

				// make subscriber
				subsc_pose = nh_ros.subscribe(node_config.topic_name_pose.value, 1000,
						&msgreader_pose_t::rosmsg_read,
						msgreader_pose.reader_pointer() );
				fprintf(stderr, "    ... ok\n");
			}
		} // <--- initialize robot pose subscriber


		// ---> initialize point-cloud subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => initialize point-cloud topic subscriber\n");

			if( !node_config.topic_name_pointcloud.value[0] ) {
				fprintf(stderr, "    ... error: laser scan topic name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_pointcloud.item);
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_pointcloud.value);

				// allocate buffer
				msgreader_pointcloud.allocate(200);

				// make subscriber
				subsc_pointcloud = nh_ros.subscribe(node_config.topic_name_pointcloud.value, 200,
						&msgreader_pointcloud_t::rosmsg_read,
						msgreader_pointcloud.reader_pointer() );
				fprintf(stderr, "    ... ok\n");
			}
		} // <--- initialize point-cloud subscriber


		if ( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, "   => initialize counting map\n" );

			if( gnd::lssmap::init_counting_map(&lssmap_counting, node_config.counting_map_cell_size.value, node_config.counting_map_cell_size.value) < 0 ) {
				ros::shutdown();
				fprintf(stderr, "    ... error: fail to create map\n");
			}
			else {
				fprintf(stderr, "    ... ok\n");
			}

		}


		// ---> text log file create
		if ( ros::ok() && node_config.text_log.value[0] ) {
			fprintf(stdout, "\n");
			fprintf(stdout, "   => create log file \"%s\"\n", node_config.text_log.value);

			if( !(fp_txtlog = fopen(node_config.text_log.value, "w")) ) {
				ros::shutdown();
				fprintf(stderr, "   ... error: fail to open\n", node_config.text_log.value);
			}
			else {
				fprintf(fp_txtlog, "#[1. sequence id] [2. x] [3. y]\n");
				fprintf(stderr, "    ... ok\n");
			}

		} // <--- text log file create

	} // <--- initialize node


	// ---> operate
	if ( ros::ok() ) {
		ros::Rate loop_rate(1000);
		ros::AsyncSpinner spinner(2);

		msg_pose_t msg_pose_prevcollect;

		double time_current;
		double time_start;
		double time_display;
		double time_collect;
		const double cycle_display = 1.0;

		int seq_pose_atcollect = 0;
		int seq_pointcloud_atcollect = 0;
		int cnt_collect = 0;

		int nline_show = 0;

		{ // ---> initialize time
			time_current = ros::Time::now().toSec();
			time_start = time_current;
			time_display = time_start;
			time_collect = time_start;
		} // <--- initialize time

		{ // ---> previous pose
			msg_pose_prevcollect.x = sqrt(DBL_MAX) / 2;
			msg_pose_prevcollect.y = sqrt(DBL_MAX) / 2;
			msg_pose_prevcollect.header.seq = 0;
			msg_pose_prevcollect.header.stamp.fromSec( time_start - node_config.collect_condition_time.value );
		} // ---> previous pose

		// ---> main loop
		spinner.start();
		while( ros::ok() ) {
			// blocking
			loop_rate.sleep();

			// time
			time_current = ros::Time::now().toSec();

			// ---> coordinate transform and publish
			if( msgreader_pose.copy_new( &msg_pose, msg_pose.header.stamp.toSec() ) == 0) {
				bool flg_collect = false;

				{ // ---> check data collect condition
					double time = msg_pose.header.stamp.toSec() - msg_pose_prevcollect.header.stamp.toSec();
					double sqdist = (msg_pose.x - msg_pose_prevcollect.x) * (msg_pose.x - msg_pose_prevcollect.x)
											+ (msg_pose.y - msg_pose_prevcollect.y) * (msg_pose.y - msg_pose_prevcollect.y);
					double angle = fabs( gnd_rad_normalize( msg_pose.theta - msg_pose_prevcollect.theta ) );

					flg_collect = flg_collect
							|| (  node_config.collect_condition_time.value > 0
									&& time >= node_config.collect_condition_time.value);
					flg_collect = flg_collect
							|| (  node_config.collect_condition_moving_distance.value > 0
									&& sqdist > node_config.collect_condition_moving_distance.value * node_config.collect_condition_moving_distance.value);
					flg_collect = flg_collect
							|| (  node_config.collect_condition_moving_angle.value > 0
									&& angle > node_config.collect_condition_moving_angle.value);
				} // <--- check data collect condition

				// ---> data collect
				if( flg_collect ) { // in meeting condition case

					// ---> coordinate transform and counting
					if( msgreader_pointcloud.copy_at_time( &msg_pointcloud, msg_pose.header.stamp.toSec()) == 0 ) { // get point cloud data
						int i;
						gnd::matrix::fixed<4,4> mat_coordtf;
						double x_src_prev, y_src_prev;

						{ // ---> initialize previous counted point
							x_src_prev = 10000;
							y_src_prev = 10000;
						} // ---> initialize previous counted point

						// calculate coordinate transform matrix
						gnd::matrix::coordinate_converter(&mat_coordtf,
								msg_pose.x, msg_pose.y, 0,
								cos(msg_pose.theta), sin(msg_pose.theta), 0,
								0, 0, 1.0);

						// ---> scanning loop (point cloud data)
						for( i = 0; i < (int)msg_pointcloud.points.size(); i++ ) {
							double sq_dist;
							gnd::vector::fixed_column<4> point_src, point_dest;

							{ // ---> ignore
								sq_dist = ( msg_pointcloud.points[i].x) * ( msg_pointcloud.points[i].x)
													+ ( msg_pointcloud.points[i].y ) * ( msg_pointcloud.points[i].y );
								if( node_config.collect_condition_ignore_range_lower.value >= 0 &&
									sq_dist < node_config.collect_condition_ignore_range_lower.value * node_config.collect_condition_ignore_range_lower.value) {
									continue;
								}
								if( node_config.collect_condition_ignore_range_upper.value >= 0 &&
									sq_dist > node_config.collect_condition_ignore_range_upper.value * node_config.collect_condition_ignore_range_upper.value) {
									continue;
								}

							} // <--- ignore

							{ // ---> culling
								sq_dist = ( msg_pointcloud.points[i].x - x_src_prev ) * ( msg_pointcloud.points[i].x - x_src_prev )
													+ ( msg_pointcloud.points[i].y - y_src_prev ) * ( msg_pointcloud.points[i].y - y_src_prev );

								if( sq_dist < node_config.collect_condition_culling_distance.value * node_config.collect_condition_culling_distance.value ) {
									continue;
								}

								x_src_prev = msg_pointcloud.points[i].x;
								y_src_prev = msg_pointcloud.points[i].y;
							} // <--- culling

							{ // ---> coordinate transform
								point_src[0] = msg_pointcloud.points[i].x;
								point_src[1] = msg_pointcloud.points[i].y;
								point_src[2] = msg_pointcloud.points[i].z;
								point_src[3] = 1;

								gnd::matrix::prod( &mat_coordtf, &point_src, &point_dest );
							} // <--- coordinate transform

							// counting
							gnd::lssmap::counting_map(&lssmap_counting, point_dest[0], point_dest[1]);

							if( fp_txtlog ) {
								fprintf( fp_txtlog, "%lf %lf\n", point_dest[0], point_dest[1] );
							}

						} // <--- scanning loop (point cloud data)

						msg_pose_prevcollect = msg_pose;

						seq_pose_atcollect = msg_pose.header.seq;
						seq_pointcloud_atcollect = msg_pointcloud.header.seq;
						cnt_collect++;
					} // <--- coordinate transform and counting

				} // <--- data collect

			} // ---> coordinate transform and publish

			// ---> status display
			if( node_config.status_display.value && time_current > time_display ) {
				// clear
				if( nline_show ) {
					fprintf(stderr, "\x1b[%02dA", nline_show);
					nline_show = 0;
				}

				nline_show++; fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_show++; fprintf(stderr, "\x1b[K operating time : %6.01lf[sec]\n", time_current - time_start);
				nline_show++; fprintf(stderr, "\x1b[K           pose : topic name \"%s\"\n", node_config.topic_name_pose.value );
				nline_show++; fprintf(stderr, "\x1b[K                :    latest seq %d\n", msg_pose.header.seq );
				nline_show++; fprintf(stderr, "\x1b[K                : collected seq %d\n", seq_pose_atcollect );
				nline_show++; fprintf(stderr, "\x1b[K    point-cloud : name \"%s\"\n", node_config.topic_name_pointcloud.value );
				nline_show++; fprintf(stderr, "\x1b[K                : collected seq  %d\n", seq_pointcloud_atcollect );
				nline_show++; fprintf(stderr, "\x1b[K                : size %d [laser points]\n", msg_pointcloud.points.size() );
				nline_show++; fprintf(stderr, "\x1b[K  collect count : %d [scans]\n", cnt_collect );

				time_display = gnd_loop_next(time_current, time_start, cycle_display);
			} // <--- status display

		} // <--- main loop
		spinner.stop();

	} // <--- operate



	{ // ---> finalize

		{ // ---> counting data file out
			gnd::lssmap::write_counting_map(&lssmap_counting, "./");
		} // <--- counting data file out

		{ // ---> build bmp image (to visualize for human)
			lssmap_t lssmap;
			gnd::bmp8_t bmp;
			gnd::bmp32_t bmp32;
			::fprintf(stdout, "  => create laser scan statistics map\n");

			// build environmental map
			gnd::lssmap::build_map(&lssmap, &lssmap_counting, node_config.sensor_range.value, node_config.additional_smoothing_parameter.value );

			// make bmp image: it show the likelihood field
			gnd::lssmap::build_bmp(&bmp, &lssmap, node_config.image_map_pixel_size.value);
			// make bmp image: it show the likelihood field
			gnd::lssmap::build_bmp(&bmp32, &lssmap, node_config.image_map_pixel_size.value);
			// file out
			gnd::bmp::write8("map-image8.bmp", &bmp);
			// file out
			gnd::bmp::write32("map-image32.bmp", &bmp32);

			gnd::lssmap::destroy_map(&lssmap);
			gnd::lssmap::destroy_counting_map(&lssmap_counting);
			{ // ---> origin
				char fname[512];
				FILE *fp = 0;
				double x, y;

				if( ::sprintf(fname, "%s", "origin.txt"  ) == sizeof(fname) ){
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open. file name is too long\n");
				}
				else if( !(fp = fopen(fname, "w")) ) {
					::fprintf(stderr, "  ... \x1b[1m\x1b[31mError\x1b[39m\x1b[0m: fail to open \"\x1b[4m%s\x1b[0m\"\n", fname);
				}
				bmp.pget_origin(&x, &y);
				fprintf(fp, "%lf %lf\n", x, y);
				fclose(fp);
			} // --->  origin


			fprintf(stdout, "   ... make map image %s\n", "map-image.bmp");
			bmp.deallocate();
			bmp32.deallocate();
		} // <--- build bmp image (to visualize for human)


		if( fp_txtlog ) {
			fclose( fp_txtlog );
		}

		fprintf(stderr, " ... fin\n");
	} // <--- finalize

	return 0;

}
