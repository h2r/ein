#ifndef MATH2D_H
#define MATH2D_H
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <assert.h>
#include "gsl_utilities.h"

#ifdef __cplusplus
extern "C" {
#endif


/* internal functions */

struct axes {
  gsl_vector * major_st;
  gsl_vector * major_end;
  gsl_vector * minor_st;
  gsl_vector * minor_end;
};


bool math2d_double_equal(double p1, double p2);

bool math2d_point_equal(gsl_vector* p1, gsl_vector* p2);

bool math2d_between(const double bound1, const double bound2, const double d);

gsl_vector* math2d_bbox(gsl_matrix* pts_xy);

gsl_matrix * math2d_bbox_to_polygon(gsl_vector * bbox);
double math2d_bbox_area(gsl_vector* bbox);

double math2d_get_scale(gsl_vector* bbox);

gsl_matrix* math2d_combined_matrix(gsl_matrix* figure, gsl_matrix* ground);


gsl_vector* math2d_closest_point_on_segment_line(gsl_vector* seg_st_xy, 
						gsl_vector* seg_end_xy, 
						gsl_vector* p_xy);

double math2d_slope(gsl_vector* seg_st_xy, gsl_vector* seg_end_xy);

bool math2d_is_on_segment(gsl_vector* seg_st_xy, gsl_vector* seg_end_xy, gsl_vector* p_xy);
bool math2d_is_on_line(gsl_matrix* line_xy, 
                       gsl_vector* p_xy) ;
bool math2d_is_on_polygon(gsl_matrix* line_xy, 
                          gsl_vector* p_xy) ;

			 
gsl_vector* math2d_closest_point_on_line(gsl_matrix* l_xy, gsl_vector* p_xy);

  gsl_vector* math2d_closest_point_on_polygon(gsl_matrix* polygon_xy, gsl_vector* p_xy);

struct axes math2d_compute_axes(gsl_matrix* ground_xy, gsl_matrix* figure_xy);

gsl_vector* math2d_intersect_segments(gsl_vector* pt1_seg1, gsl_vector* pt2_seg1, 
                                      gsl_vector* pt1_seg2, gsl_vector* pt2_seg2, bool bound);

gsl_matrix* math2d_perpendicular_segment(gsl_vector* seg_st, gsl_vector* seg_end, gsl_vector* start_point);

gsl_vector* math2d_closest_point_on_segment(gsl_vector* seg_st_xy, gsl_vector* seg_end_xy, gsl_vector* p_xy);

gsl_matrix* math2d_intersect_polygon_line_analytic(gsl_matrix* polygon, double m, double b);
gsl_matrix* math2d_intersect_polygon_line(gsl_matrix* polygon_xy, gsl_matrix* line_xy);

gsl_matrix * math2d_polygon_to_line(gsl_matrix * polygon);

gsl_matrix * math2d_intersect_lines(gsl_matrix * line1_xy, gsl_matrix * line2_xy);
gsl_vector* math2d_intersect_line_segment_line(gsl_vector* pt1_seg, gsl_vector* pt2_seg, double m, double b);

bool math2d_is_interior_point(gsl_vector* pt, gsl_matrix* polygon_xy);

gsl_vector* math2d_line_equation(gsl_vector* pt1, gsl_vector* pt2);

gsl_matrix* math2d_compute_boundary_line(gsl_matrix* landmark_xy, gsl_matrix* figure_xy) ;


inline double math2d_square_dist(gsl_vector* pt1, gsl_vector* pt2)
{
  double x1 = gsl_vector_get(pt1, 0);
  double y1 = gsl_vector_get(pt1, 1);
  double x2 = gsl_vector_get(pt2, 0);
  double y2 = gsl_vector_get(pt2, 1);
  double xdiff = x2 - x1;
  double ydiff = y2 - y1;
  return xdiff * xdiff + ydiff * ydiff;
}

inline double math2d_dist(gsl_vector* pt1, gsl_vector* pt2)
{
  return sqrt(math2d_square_dist(pt1, pt2));
}

double math2d_line_length(gsl_matrix * line_xy) ;
gsl_matrix * math2d_step_along_line(gsl_matrix * line_xy, double step_size) ;
gsl_matrix * math2d_step_along_polygon(gsl_matrix * polygon, double step_size);

double math2d_angle_between_segments(gsl_vector * s1_start, gsl_vector * s1_end, gsl_vector * s2_start, gsl_vector * s2_end);

double math2d_perimeter(gsl_matrix * polygon_xy);
double math2d_dist_between_points_along_line(gsl_matrix * line, gsl_vector * p1, gsl_vector * p2);
double math2d_dist_between_points_along_polygon(gsl_matrix * polygon, gsl_vector * p1, gsl_vector * p2);
gsl_matrix* math2d_trim_polygon(gsl_matrix * polygon_xy, gsl_vector * p1, gsl_vector * p2) ;
gsl_matrix * math2d_trim_line(gsl_matrix * line_xy, gsl_vector * p1, gsl_vector * p2) ;

struct eigenstuff {
  gsl_vector * evals;
  gsl_matrix * evecs;
};
void math2d_free_eigenstuff(struct eigenstuff estuff); 
struct eigenstuff math2d_eigenvectors(gsl_matrix * polygon);

gsl_vector * math2d_centroid(gsl_matrix * polygon);
double math2d_signed_area(gsl_matrix * polygon);
double math2d_area(gsl_matrix * polygon);
struct axes math2d_eigen_axes(gsl_matrix * polygon);
void math2d_axes_free(struct axes axes);

struct fit_line_result {
  double slope;
  double intercept;
};
struct fit_line_result math2d_fit_line(gsl_matrix * points);
void math2d_set_point(gsl_vector * point, double x, double y);
gsl_vector * math2d_point(double x, double y);
gsl_vector * math2d_center_of_mass(gsl_matrix * points);
void math2d_points_printf(gsl_matrix * points);
void math2d_vector_printf(gsl_vector * points) ;
double math2d_vector_sum(gsl_vector* vec, int start_idx, int end_idx);

struct math2d_range {
  int start_i;
  int end_i;
};

struct math2d_range math2d_smallest_window(gsl_vector * lst, int window_size);
void math2d_point_on_segment(gsl_vector * seg_st_xy, gsl_vector * seg_end_xy, double distance, gsl_vector * result) ;

gsl_matrix * math2d_top(gsl_matrix * polygon, gsl_vector * direction_xy);
gsl_vector * math2d_vector_to_unit_vector(gsl_vector * direction_xy);

bool math2d_is_visible(gsl_matrix * polygon, gsl_vector * p1, gsl_vector * p2);
gsl_vector * math2d_lowest_point(gsl_matrix * points, 
                                 gsl_vector * direction_xy);
gsl_vector * math2d_highest_point(gsl_matrix * points, 
                                  gsl_vector * direction_xy);
double math2d_height_in_direction(gsl_vector * point, 
                                  gsl_vector * direction);

double math2d_point_length(gsl_vector * point); 

double math2d_angle_between_points(gsl_vector * p1, 
                                   gsl_vector * p2);

bool math2d_overlaps(gsl_matrix * polygon1, gsl_matrix * polygon2);

gsl_vector * math2d_midpoint_segment(gsl_vector * p1, gsl_vector * p2);

int math2d_cmp_points(const gsl_vector * p1, const gsl_vector * p2);
int math2d_cmp(double v1, double v2); 
gsl_matrix * math2d_sort_points(gsl_matrix * points);
double math2d_angle(gsl_vector * vector);
gsl_vector * math2d_rotate(gsl_vector * vector,
                           double theta);

bool math2d_range_overlaps(double r1_s, double r1_e, double r2_s, double r2_e);
gsl_matrix * math2d_copy_point_list_up_to(gsl_matrix * source, int copy_to_idx);

int math2d_angle_to_quadrant(double theta) ;
int math2d_angle_to_octant(double theta) ;


#ifdef __cplusplus
}
#endif


#endif

