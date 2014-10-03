/*
 * gnd_lssmap_maker_config.hpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 *       Brief: Laser Scan Statistics MAP MAKER node CONFIGuration definition
 */

#ifndef GND_LSSMAP_MAKER_CONFIG_HPP_
#define GND_LSSMAP_MAKER_CONFIG_HPP_

#include <string.h>

#include "gnd/gnd-multi-math.h"

#include "gnd/gnd-util.h"
#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-lib-error.h"


// ---> type declaration
namespace gnd {
	namespace lssmap_maker {
		struct node_config;
		typedef struct node_config node_config;

		typedef gnd::conf::parameter_array<char, 256> param_string_t;
		typedef gnd::conf::param_int param_int_t;
		typedef gnd::conf::param_long param_long_t;
		typedef gnd::conf::param_double param_double_t;
		typedef gnd::conf::param_bool param_bool_t;
	}
} // <--- type declaration



// ---> const variables definition
namespace gnd {
	namespace lssmap_maker {

		// ---> ros communication
		static const param_string_t Default_node_name = {
				"node-name",
				"lssmap_maker",
				"ros-node name"
		};

		static const param_string_t Default_topic_name_pose = {
				"topic-pose",
				"pose-gl",
				"global pose topic (subscribe)"
		};

		static const param_string_t Default_topic_name_pointcloud = {
				"topic-laserscan-point",
				"foo_pointcloud",
				"laser scan point topic, type PointCloud on robot coordinate (subscribe)"
		};
		// <--- ros communication


		// ---> map option
		static const param_double_t Default_counting_map_cell_size = {
				"counting-map-cell-size",
				gnd_cm2m(80),
				"cell size for counting to calculate variance and means (m)"
		};

		static const param_double_t Default_image_map_pixel_size = {
				"image-map-pixel-size",
				gnd_cm2m(10),
				"image map pixel size (m)"
		};

		static const param_double_t Default_additional_smoothing_parameter = {
				"additional-smoothing-parameter",
				100,
				"additional smoothing parameter"
		};

		static const param_double_t Default_sensor_range = {
				"sensor-range",
				30,
				"sensor range (m)"
		};
		// <--- map option



		// ---> data collect option
		static const param_double_t Default_collect_condition_ignore_range_lower= {
				"collect-condition-ignore-range-lower",
				gnd_cm2m(20),
				"ignore laser scan data (m)"
		};

		static const param_double_t Default_collect_condition_ignore_range_upper = {
				"collect-condition-ignore-range-upper",
				-1,
				"ignore laser scan data (m)"
		};

		static const param_double_t Default_collect_condition_culling_distance = {
				"collect-condition-culling-distance",
				gnd_cm2m(5),
				"distance condition to ignore data in laser scan data counting (m)"
		};

		static const param_double_t Default_collect_condition_moving_distance = {
				"collect-condition-moving-distance",
				gnd_cm2m(5),
				"data collect condition of moving distance(m), [note] if this parameter is <=0, not collect data under the moving distance condition"
		};

		static const param_double_t Default_collect_condition_moving_angle = {
				"collect-condition-moving-angle",
				gnd_deg2ang(90),
				"data collect condition of moving angle(deg), [note] if this parameter is <=0, not collect data under the moving angle condition"
		};

		static const param_double_t Default_collect_condition_time = {
				"collect-condition-time",
				0,
				"data collect condition (sec), [note] if this parameter is <=0, not collect data under the time condition"
		};
		// <--- data collect option



		// ---> debug condition
		static const param_double_t Default_cycle_status_display = {
				"cycle-cui-status-display",
				0,
				"cycle to display the node status in standard output [msec]. [note] it need ansi color code. [note] if this value is less than or equal 0, don't display"
		};

		static const param_string_t Default_text_log = {
				"text-log",
				"",
				"text log file name. this file collect point cloud data. [note] if this parameter is null, the file is not created."
		};
		// <--- debug condition
	}
}
// <--- const variables definition



// ---> function declaration
namespace gnd {
	namespace lssmap_maker {
		/**
		 * @brief initialize configure to default parameter
		 * @param [out] p : node_config
		 */
		int init_node_config(node_config *conf);


		/**
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		int fread_node_config( const char* fname, node_config *dest );
		/**
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		int get_node_config( node_config *dest, gnd::conf::configuration *src );



		/**
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		int fwrite_node_config( const char* fname, node_config *src );
		/**
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		int set_node_config( gnd::conf::configuration *dest, node_config *src );
	}
}
// ---> function declaration



// ---> type definition
namespace gnd {
	namespace lssmap_maker {
		/**
		 * @brief configuration parameter for gnd_urg_proxy node
		 */
		struct node_config {

			node_config();
			// ros communication option
			param_string_t node_name;							///< node name for ros communication
			param_string_t topic_name_pose;						///< pose topic name for ros communication
			param_string_t topic_name_pointcloud;				///< pointcloud topic name for ros communication
			// map make option
			param_double_t counting_map_cell_size;				///< counting cell size
			param_double_t image_map_pixel_size;				///< image map pixel size
			param_double_t additional_smoothing_parameter;		///< additional smoothing parameter
			param_double_t sensor_range;						///< sensor range
			// data collect option
			param_double_t collect_condition_ignore_range_lower;///< ignore range
			param_double_t collect_condition_ignore_range_upper;///< ignore upper
			param_double_t collect_condition_culling_distance;	///< culling distance
			param_double_t collect_condition_moving_distance;	///< data collect condition (moving distance)
			param_double_t collect_condition_moving_angle;		///< data collect condition (moving angle)
			param_double_t collect_condition_time;				///< data collect condition (time)
			// debug option
			param_double_t cycle_cui_status_display;			///< cui status display mode
			param_string_t text_log;							///< text log file name
		};


		inline
		node_config::node_config() {
			init_node_config(this);
		}
	}
}
// <--- type definition



// ---> function definition
namespace gnd {
	namespace lssmap_maker {
		/*
		 * @brief initialize configuration parameter
		 * @param [out] p : node_config
		 */
		inline
		int init_node_config( node_config *p ){
			gnd_assert(!p, -1, "invalid null pointer argument\n" );

			// ros communication option
			memcpy( &p->node_name,								&Default_node_name,								sizeof(Default_node_name) );
			memcpy( &p->topic_name_pose,						&Default_topic_name_pose,						sizeof(Default_topic_name_pose) );
			memcpy( &p->topic_name_pointcloud,					&Default_topic_name_pointcloud,					sizeof(Default_topic_name_pointcloud) );
			// map make option
			memcpy( &p->counting_map_cell_size,					&Default_counting_map_cell_size,				sizeof(Default_counting_map_cell_size) );
			memcpy( &p->image_map_pixel_size,					&Default_image_map_pixel_size,					sizeof(Default_image_map_pixel_size) );
			memcpy( &p->additional_smoothing_parameter,			&Default_additional_smoothing_parameter,		sizeof(Default_additional_smoothing_parameter) );
			memcpy( &p->sensor_range,							&Default_sensor_range,							sizeof(Default_sensor_range) );
			memcpy( &p->collect_condition_ignore_range_lower,	&Default_collect_condition_ignore_range_lower,	sizeof(Default_collect_condition_ignore_range_lower) );
			memcpy( &p->collect_condition_ignore_range_upper,	&Default_collect_condition_ignore_range_upper,	sizeof(Default_collect_condition_ignore_range_upper) );
			memcpy( &p->collect_condition_culling_distance,		&Default_collect_condition_culling_distance,	sizeof(Default_collect_condition_culling_distance) );
			memcpy( &p->collect_condition_moving_distance,		&Default_collect_condition_moving_distance,		sizeof(Default_collect_condition_moving_distance) );
			memcpy( &p->collect_condition_moving_angle,			&Default_collect_condition_moving_angle,		sizeof(Default_collect_condition_moving_angle) );
			memcpy( &p->collect_condition_time,					&Default_collect_condition_time,				sizeof(Default_collect_condition_time) );
			// debug option
			memcpy( &p->cycle_cui_status_display,				&Default_cycle_status_display,					sizeof(Default_cycle_status_display) );
			memcpy( &p->text_log,								&Default_text_log,								sizeof(Default_text_log) );

			return 0;
		}

		/*
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int fread_node_config( const char* fname, node_config *dest ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(fname)) < 0 )    return ret;

				return get_node_config(dest, &fs);
			} // <--- operation
		}
		/*
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		inline
		int get_node_config( node_config *dest, gnd::conf::configuration *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication option
			gnd::conf::get_parameter( src, &dest->node_name );
			gnd::conf::get_parameter( src, &dest->topic_name_pose );
			gnd::conf::get_parameter( src, &dest->topic_name_pointcloud );
			// map maker option
			gnd::conf::get_parameter( src, &dest->counting_map_cell_size );
			gnd::conf::get_parameter( src, &dest->image_map_pixel_size );
			gnd::conf::get_parameter( src, &dest->additional_smoothing_parameter );
			gnd::conf::get_parameter( src, &dest->sensor_range );
			// data collect option
			gnd::conf::get_parameter( src, &dest->collect_condition_ignore_range_lower );
			gnd::conf::get_parameter( src, &dest->collect_condition_ignore_range_upper );
			gnd::conf::get_parameter( src, &dest->collect_condition_culling_distance );
			gnd::conf::get_parameter( src, &dest->collect_condition_moving_distance );
			if( gnd::conf::get_parameter( src, &dest->collect_condition_moving_angle ) >= 0) {
				dest->collect_condition_moving_angle.value = gnd_deg2ang(dest->collect_condition_moving_angle.value);
			}
			gnd::conf::get_parameter( src, &dest->collect_condition_time );
			// debug option
			gnd::conf::get_parameter( src, &dest->cycle_cui_status_display );
			gnd::conf::get_parameter( src, &dest->text_log );

			return 0;
		}



		/*
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		inline
		int fwrite_node_config( const char* fname, node_config *src ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );
			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

				return fs.write(fname);
			} // <--- operation
		}

		/*
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		inline
		int set_node_config( gnd::conf::configuration *dest, node_config *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication option
			gnd::conf::set_parameter( dest, &src->node_name );
			gnd::conf::set_parameter( dest, &src->topic_name_pose );
			gnd::conf::set_parameter( dest, &src->topic_name_pointcloud );
			// map maker option
			gnd::conf::set_parameter( dest, &src->counting_map_cell_size );
			gnd::conf::set_parameter( dest, &src->image_map_pixel_size );
			gnd::conf::set_parameter( dest, &src->additional_smoothing_parameter );
			gnd::conf::set_parameter( dest, &src->sensor_range );
			// data collect option
			gnd::conf::set_parameter( dest, &src->collect_condition_ignore_range_lower );
			gnd::conf::set_parameter( dest, &src->collect_condition_ignore_range_upper );
			gnd::conf::set_parameter( dest, &src->collect_condition_culling_distance );
			gnd::conf::set_parameter( dest, &src->collect_condition_moving_distance );
			{
				param_double_t ws;
				memcpy(&ws, &src->collect_condition_moving_angle, sizeof(ws));
				ws.value = gnd_ang2deg(ws.value);
				gnd::conf::set_parameter( dest, &ws );
			}
			gnd::conf::set_parameter( dest, &src->collect_condition_time );
			// debug option
			gnd::conf::set_parameter( dest, &src->cycle_cui_status_display );
			gnd::conf::set_parameter( dest, &src->text_log );

			return 0;
		}

	}
}
// <--- function definition


#endif /* GND_LSSMAP_MAKER_CONFIG_HPP_ */
