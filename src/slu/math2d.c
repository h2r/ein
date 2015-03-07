#include "math2d.h"
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_fit.h>
#include <stdlib.h>

double math2d_equal_threshold = 0.0001;
gsl_vector * origin = math2d_point(0, 0);

struct axes math2d_empty_axes() 
{
  struct axes result;
  result.major_st = NULL;
  result.major_end = NULL;
  result.minor_st = NULL;
  result.minor_end = NULL;
  return result;
}

void math2d_axes_free(struct axes in) 
{
  gsl_vector_free(in.major_st);
  gsl_vector_free(in.major_end);
  gsl_vector_free(in.minor_st);
  gsl_vector_free(in.minor_end);

}

void math2d_set_point(gsl_vector * ret, double x, double y)
{
  gsl_vector_set(ret, 0, x);
  gsl_vector_set(ret, 1, y);
}

gsl_vector * math2d_point(double x, double y)
{
  gsl_vector * result = gsl_vector_alloc(2);
  math2d_set_point(result, x, y);
  return result;
}

gsl_matrix * math2d_copy_point_list_up_to(gsl_matrix * source, int copy_to_idx) 
{

  gsl_matrix * result = gsl_matrix_alloc(source->size1, copy_to_idx);
  for (int i = 0; i < copy_to_idx; i++) {
    gsl_matrix_set_col(result, i, &(gsl_matrix_column(source, i).vector));
  }
  return result;

}



gsl_matrix * math2d_point_list_append(gsl_matrix * l1, gsl_matrix * l2)
{

  gsl_matrix * result = gsl_matrix_alloc(l1->size1, l1->size2 + l2->size2);
  
  for (size_t i = 0; i < l1->size2; i++) {
    gsl_matrix_set_col(result, i, &(gsl_matrix_column(l1, i).vector));
  }
  for (size_t i = 0; i < l2->size2; i++) {
    gsl_matrix_set_col(result, i + l1->size2, &(gsl_matrix_column(l2, i).vector));
  }

  return result;

}


gsl_matrix * math2d_matrix_copy(gsl_matrix * source)
{

  gsl_matrix * result = gsl_matrix_alloc(source->size1, source->size2);
  gsl_matrix_memcpy(result, source);
  return result;
}

gsl_vector * math2d_vector_copy(gsl_vector * source)
{

  gsl_vector * result = gsl_vector_alloc(source->size);
  gsl_vector_memcpy(result, source);
  return result;
}


bool math2d_is_degenerate(gsl_vector * s1_start, gsl_vector * s1_end)
{
  return math2d_point_equal(s1_start, s1_end);
}

bool math2d_is_vertical(gsl_vector * s1_start, gsl_vector * s1_end)
{
  double x1 = gsl_vector_get(s1_start, 0);
  double y1 = gsl_vector_get(s1_start, 1);

  double x2 = gsl_vector_get(s1_end, 0);
  double y2 = gsl_vector_get(s1_end, 1);

  return math2d_double_equal(x1, x2) && ! math2d_double_equal(y1, y2);
}

int math2d_angle_to_quadrant(double theta) 
{
  theta = tklib_normalize_theta(theta);
  if (theta < 0) {
    theta += M_PI * 2;
  }
  
  if ((0 <= theta) &&  theta < M_PI/2) {
    return 0;
  } else if ((M_PI/2 <= theta) && (theta < M_PI)) {
    return 1;
  } else if ((M_PI <= theta) && (theta < 3 * M_PI/2)) {
    return 2;
  } else if ((3 * M_PI/2 <= theta) && (theta <= 2 * M_PI)) {
    return 3;
  } else {
    printf("Should never get here; theta is bad: %10f\n", theta);
    assert(0);
  }
}

int math2d_angle_to_octant(double theta) 
{
  theta = tklib_normalize_theta(theta);
  if (theta < 0) {
    theta += M_PI * 2;
  }
  
  if ((0 <= theta) &&  theta < M_PI/4) {
    return 0;
  } else if ((1 * M_PI/4 <= theta) && (theta < 2 * M_PI/4)) {
    return 1;
  } else if ((2 * M_PI/4 <= theta) && (theta < 3 * M_PI/4)) {
    return 2;
  } else if ((3 * M_PI/4 <= theta) && (theta < 4 * M_PI/4)) {
    return 3;
  } else if ((4 * M_PI/4 <= theta) && (theta < 5 * M_PI/4)) {
    return 4;
  } else if ((5 * M_PI/4 <= theta) && (theta < 6 * M_PI/4)) {
    return 5;
  } else if ((6 * M_PI/4 <= theta) && (theta < 7 * M_PI/4)) {
    return 6;
  } else if ((7 * M_PI/4 <= theta) && (theta <= 8 * M_PI/4)) {
    return 7;
  } else {
    printf("Should never get here; theta is bad: %10f\n", theta);
    assert(0);
  }
}


double math2d_angle_between_points(gsl_vector * p1, 
                                   gsl_vector * p2)
{
  return acos(tklib_vector_dot(p1, p2) / math2d_point_length(p1) * 
              math2d_point_length(p2));
}

/**
   """
   Always returns an angle between 0 ando pi/2
   """
*/
double math2d_angle_between_segments(gsl_vector * s1_start, gsl_vector * s1_end,
                                     gsl_vector * s2_start, gsl_vector * s2_end)
{
  if (math2d_is_degenerate(s1_start, s1_end) || math2d_is_degenerate(s2_start, s2_end)) {
    return 0;
  } else if (math2d_is_vertical(s1_start, s1_end) && math2d_is_vertical(s2_start, s2_end)) {
    return 0;
  } else if (math2d_is_vertical(s1_start, s1_end) || math2d_is_vertical(s2_start, s2_end)) {

    gsl_vector * unit_horizontal = math2d_point(1, 0);

    double result;
    if (math2d_is_vertical(s1_start, s1_end)) {
      result = M_PI/2 - math2d_angle_between_segments(s2_start, s2_end, origin, unit_horizontal);
    } else {
      result = M_PI/2 - math2d_angle_between_segments(s1_start, s1_end, origin, unit_horizontal);
    } 
    gsl_vector_free(unit_horizontal);
    return result;
  } else {
    double m1 = math2d_slope(s1_start, s1_end);
    double m2 = math2d_slope(s2_start, s2_end);
    
    double angle = fabs(atan2(m2 - m1, 1 + m1*m2));
    if (angle > M_PI/2) {
      angle = M_PI - angle;
    }
    assert(0 <= angle);
    assert(angle <= M_PI/2);
    return angle;
  }
}

gsl_matrix * math2d_step_along_polygon(gsl_matrix * polygon, double step_size) 
{
  gsl_matrix * line = math2d_polygon_to_line(polygon);
  gsl_matrix * result = math2d_step_along_line(line, step_size);
  gsl_matrix_free(line);
  return result;
}


gsl_matrix * math2d_step_along_line(gsl_matrix * line_xy, double step_size) 
{
  double dist_along_step = 0;
  double line_length = math2d_line_length(line_xy);

  if (math2d_double_equal(step_size, 0) || 
      math2d_double_equal(line_length, 0)) {
    return math2d_matrix_copy(line_xy);
  }
  int result_length = (int) ((line_length / step_size) + 1) * 2;
  gsl_matrix * result = gsl_matrix_alloc(2, result_length);
                   
  
  gsl_vector_view line_st = gsl_matrix_column(line_xy, 0);
  gsl_vector_view line_end = gsl_matrix_column(line_xy, line_xy->size2 - 1);
  gsl_vector * yield_p = gsl_vector_alloc(2);
  int idx = 0;
  gsl_matrix_set_col(result, idx, &line_st.vector);
  idx++;

  for (size_t i = 0; i < line_xy->size2 - 1; i++) {
    gsl_vector_view p1 = gsl_matrix_column(line_xy, i);
    gsl_vector_view p2 = gsl_matrix_column(line_xy, i+1);
    gsl_vector * start_p = &(p1.vector);
    while (true) {
      double new_dist_along_step = dist_along_step + 
        math2d_dist(start_p, &p2.vector);
      if (new_dist_along_step >= step_size) {
        math2d_point_on_segment(start_p, &p2.vector, step_size - dist_along_step, yield_p);

        gsl_matrix_set_col(result, idx, yield_p);
        idx++;
        start_p = yield_p;
        dist_along_step = 0.0;
      } else {
        dist_along_step = new_dist_along_step;
        break;
      }
    }
  }


  gsl_matrix * real_result = math2d_copy_point_list_up_to(result, idx);
  gsl_vector_free(yield_p);
  gsl_matrix_free(result);
  return real_result;
}

bool math2d_point_equal(gsl_vector* p1, gsl_vector* p2){
  double d = math2d_dist(p1, p2);
  
  if(d < math2d_equal_threshold*p1->size)
    return true;
  
  return false;
}

bool math2d_double_equal(double p1, double p2){
  return fabs(p1 - p2) < math2d_equal_threshold;
}


bool math2d_between(const double bound1, const double bound2, const double d){
  return (((bound1 <= d || math2d_double_equal(bound1, d)) && 
	   (d <= bound2 || math2d_double_equal(bound2, d))) || 
	  ((bound2 <= d || math2d_double_equal(bound2, d)) && 
	   (d <= bound1 || math2d_double_equal(bound1, d))));
}


/**
 * Convert the bounding box (xmin, ymin, xmax, ymax) to a polygon,
 * representing it as a list of vertices.
 */
gsl_matrix * math2d_bbox_to_polygon(gsl_vector * bbox)
{
  double min_x = gsl_vector_get(bbox, 0);
  double min_y = gsl_vector_get(bbox, 1);
  double max_x = gsl_vector_get(bbox, 2);
  double max_y = gsl_vector_get(bbox, 3);
  double width = max_x - min_x;
  double height = max_y - min_y;
  
  gsl_matrix * polygon = gsl_matrix_alloc(2, 4);
  gsl_matrix_set(polygon, 0, 0, min_x);
  gsl_matrix_set(polygon, 1, 0, min_y);

  gsl_matrix_set(polygon, 0, 1, min_x + width);
  gsl_matrix_set(polygon, 1, 1, min_y);

  gsl_matrix_set(polygon, 0, 2, min_x + width);
  gsl_matrix_set(polygon, 1, 2, min_y + height);

  gsl_matrix_set(polygon, 0, 3, min_x);
  gsl_matrix_set(polygon, 1, 3, min_y + height);
  return polygon;
}


/**
 * Find the bounding box for the list of points.
 * The bounding box is represented as xmin, ymin, xmax, ymax.
 */
gsl_vector* math2d_bbox(gsl_matrix* pts_xy){
  double xmax = GSL_NEGINF;
  double ymax = GSL_NEGINF;
  double xmin = GSL_POSINF; 
  double ymin = GSL_POSINF;

  for (size_t i = 0; i < pts_xy->size2; i++) {
    double x = gsl_matrix_get(pts_xy, 0, i);
    double y = gsl_matrix_get(pts_xy, 1, i);
    if (x > xmax) {
      xmax = x;
    }
    if (y > ymax) {
      ymax = y;
    }
    if (x < xmin) {
      xmin = x;
    }
    if (y < ymin) {
      ymin = y;
    }
  }
  double scaleX = xmax - xmin;
  double scaleY = ymax - ymin;
  assert(scaleX >= 0);
  assert(scaleY >= 0);


  gsl_vector* boundingbox = gsl_vector_alloc(4);
  gsl_vector_set(boundingbox, 0, xmin);
  gsl_vector_set(boundingbox, 1, ymin);
  gsl_vector_set(boundingbox, 2, xmax);
  gsl_vector_set(boundingbox, 3, ymax);

  return boundingbox;
}


double math2d_get_scale(gsl_vector* bbox){
  /* get the width and height from the bounding box */
  double width = gsl_vector_get(bbox, 2) - gsl_vector_get(bbox, 0);
  double height = gsl_vector_get(bbox, 3) - gsl_vector_get(bbox, 1);
  
  /* get the scale */
  double scale = sqrt(width*width + height*height);
  
  return scale;
}



double math2d_bbox_area(gsl_vector* bbox)
{
  double width = gsl_vector_get(bbox, 2) - gsl_vector_get(bbox, 0);
  double height = gsl_vector_get(bbox, 3) - gsl_vector_get(bbox, 1);
  return width * height;
}

gsl_matrix* math2d_combined_matrix(gsl_matrix* figure, gsl_matrix* ground){
  gsl_matrix* combined = gsl_matrix_alloc(figure->size1, figure->size2+ground->size2);
  
  gsl_matrix_view fig_sub = gsl_matrix_submatrix(combined, 0, 0, figure->size1, figure->size2);
  gsl_matrix_view gnd_sub = gsl_matrix_submatrix(combined, 0, figure->size2, 
						 figure->size1, ground->size2);
  
  gsl_matrix_memcpy(&fig_sub.matrix, figure);
  gsl_matrix_memcpy(&gnd_sub.matrix, ground);

  /* get the bounding box and then its width and height */
  //gsl_vector* bbox = math2d_bounding_box(combined);
  //gsl_matrix_free(combined);
  
  return combined;
}






/**
 * 
 * Find a point a given distance along a line segment
 * 
*/
void math2d_point_on_segment(gsl_vector * seg_st_xy, gsl_vector * seg_end_xy, double distance, 
                             gsl_vector * result) 
{
  double x1 = gsl_vector_get(seg_st_xy, 0);
  double y1 = gsl_vector_get(seg_st_xy, 1);
  double x2 = gsl_vector_get(seg_end_xy, 0);
  double y2 = gsl_vector_get(seg_end_xy, 1);
  double segment_length = math2d_dist(seg_st_xy, seg_end_xy);
  double cos_angle = (x2 - x1)/segment_length;
  double sin_angle = (y2 - y1)/segment_length;

  double xdist = cos_angle * distance;
  double ydist = sin_angle * distance;

  math2d_set_point(result, x1 + xdist, y1 + ydist);
}

gsl_vector* math2d_closest_point_on_segment_line(gsl_vector* seg_st_xy, 
						gsl_vector* seg_end_xy, 
						gsl_vector* p_xy)
{
  if (math2d_dist(seg_st_xy, seg_end_xy) < 10e-5){
    gsl_vector* ret_vec = gsl_vector_alloc(seg_st_xy->size);
    gsl_vector_memcpy(ret_vec, seg_st_xy);
    return ret_vec;
 }

  double x1, y1, x2, y2;
  if (gsl_vector_get(seg_st_xy, 0) < gsl_vector_get(seg_end_xy, 0)) {
    x1 = gsl_vector_get(seg_st_xy, 0); //s.start.x;
    y1 = gsl_vector_get(seg_st_xy, 1); //s.start.y;
    x2 = gsl_vector_get(seg_end_xy, 0); //s.end.x;
    y2 = gsl_vector_get(seg_end_xy, 1); //s.end.y;
  } else {
    x2 = gsl_vector_get(seg_st_xy, 0); //s.start.x;
    y2 = gsl_vector_get(seg_st_xy, 1); //s.start.y;
    x1 = gsl_vector_get(seg_end_xy, 0); //s.end.x;
    y1 = gsl_vector_get(seg_end_xy, 1); //s.end.y;
  }
  double u = (((gsl_vector_get(p_xy, 0) - x1) * (x2 - x1) + (gsl_vector_get(p_xy, 1) - y1) * (y2 - y1)) / 
	      pow(math2d_dist(seg_st_xy, seg_end_xy),2.0));
  
  gsl_vector* ret = gsl_vector_alloc(2);
  gsl_vector_set(ret, 0, x1 + u * (x2 - x1));
  gsl_vector_set(ret, 1, y1 + u * (y2 - y1));
  return ret;
}

double math2d_slope(gsl_vector* seg_st_xy, gsl_vector* seg_end_xy){
  double num = gsl_vector_get(seg_end_xy, 1) - gsl_vector_get(seg_st_xy, 1);
  double denom = gsl_vector_get(seg_end_xy, 0) - gsl_vector_get(seg_st_xy, 0);
  
  if(denom == 0)
    return GSL_POSINF;
  //return (s.end.y - s.start.y) / (s.end.x - s.start.x); // POSSIBLE DIVIDE BY ZERO
  return num/denom;
}

bool math2d_is_on_polygon(gsl_matrix* line_xy, 
                          gsl_vector* p_xy) 
{

  gsl_matrix * polygon = math2d_polygon_to_line(line_xy);
  bool result = math2d_is_on_line(polygon, p_xy);
  gsl_matrix_free(polygon);
  return result;
}

bool math2d_is_on_line(gsl_matrix* line_xy, 
                      gsl_vector* p_xy) 
{
  
  for (size_t i = 0; i < line_xy->size2 - 1; i++) {
    gsl_vector_view seg_start_xy = gsl_matrix_column(line_xy, i);
    gsl_vector_view seg_end_xy = gsl_matrix_column(line_xy, i+1);
    if (math2d_is_on_segment(&seg_start_xy.vector, &seg_end_xy.vector, p_xy)) {
      return true;
    }
  }
  return false;
}

bool math2d_is_on_segment(gsl_vector* seg_st_xy, 
                          gsl_vector* seg_end_xy, 
                          gsl_vector* p_xy){

  /* if the start location of the segment are equal
     then return true only if the start location is the 
     same as the point*/ 
  if (math2d_point_equal(seg_st_xy, seg_end_xy))
    return math2d_point_equal(seg_st_xy, p_xy);
  /*if its vertical, then do something special */
  else if(math2d_double_equal(gsl_vector_get(seg_st_xy, 0), 
			     gsl_vector_get(seg_end_xy, 0))){ 
    
    /* If the x location of the point is on the segment and the y location is between the segment*/
    //s.start.y, s.end.y, p.y))
    if (math2d_double_equal(gsl_vector_get(p_xy, 0), gsl_vector_get(seg_st_xy, 0))
	&& math2d_between(gsl_vector_get(seg_st_xy, 1), gsl_vector_get(seg_end_xy, 1), gsl_vector_get(p_xy, 1)))
      return true;
    else 
      return false;
  } 
  else {
    double m = math2d_slope(seg_st_xy, seg_end_xy);
    //s.start.y - s.start.x * m;
    double b = gsl_vector_get(seg_st_xy, 1)-gsl_vector_get(seg_st_xy, 0) * m;
    
    //p.x*m + b, p.y)) {
    if (!math2d_double_equal(gsl_vector_get(p_xy, 0)*m + b, gsl_vector_get(p_xy, 1)))
      return false;
    //s.start.x, s.end.x, p.x);
    return math2d_between(gsl_vector_get(seg_st_xy, 0), gsl_vector_get(seg_end_xy, 0), gsl_vector_get(p_xy, 0));
  }
}



gsl_vector* math2d_closest_point_on_segment(gsl_vector* seg_st_xy, 
					   gsl_vector* seg_end_xy, 
					   gsl_vector* p_xy){
  gsl_vector* candidate = math2d_closest_point_on_segment_line(seg_st_xy, seg_end_xy, p_xy);

  if (!math2d_is_on_segment(seg_st_xy, seg_end_xy, candidate)) {
    //s.start, p);
    //s.end, p);
    double d1 = math2d_dist(seg_st_xy, p_xy);
    double d2 = math2d_dist(seg_end_xy, p_xy);
    //return s.end;
    if (d1 > d2) 
      gsl_vector_memcpy(candidate, seg_end_xy);
    //return s.start;
    else
      gsl_vector_memcpy(candidate, seg_st_xy);
  }
  
  return candidate;
}

gsl_vector* math2d_closest_point_on_line(gsl_matrix* l_xy, gsl_vector* p_xy) // DANGER
{
  double best_dist = -1;
  assert(l_xy->size2>0);
  //l.points[0];
  
  gsl_vector* best_point = gsl_vector_alloc(l_xy->size1);
  gsl_matrix_get_col(best_point, l_xy, 0);
  //gsl_vector* seg_st_xy = gsl_vector_alloc(2);
  //gsl_vector* seg_end_xy = gsl_vector_alloc(2);
  
  for (size_t i = 0; i < l_xy->size2 - 1; i++) {
    gsl_vector_view seg_st_xy = gsl_matrix_column(l_xy, i);//.points[i];
    gsl_vector_view seg_end_xy = gsl_matrix_column(l_xy, i+1);//.points[i+1];
    
    //s.start = p1;
    //s.end = p2;
    gsl_vector* tmp_point = math2d_closest_point_on_segment(&seg_st_xy.vector, 
							   &seg_end_xy.vector, p_xy);
    double curr_d = math2d_dist(tmp_point, p_xy);
    
    if (best_dist == -1 || curr_d < best_dist) {
      best_dist = curr_d;
      gsl_vector_memcpy(best_point, tmp_point);
    }
    
    gsl_vector_free(tmp_point);
  }
  return best_point;
}


gsl_vector* math2d_closest_point_on_polygon(gsl_matrix* polygon_xy, gsl_vector* p_xy)
{
  gsl_matrix * line = math2d_polygon_to_line(polygon_xy);
  gsl_vector * best_point = math2d_closest_point_on_line(line, p_xy);
  gsl_matrix_free(line);
  return best_point;
}

gsl_matrix * math2d_polygon_to_line(gsl_matrix * polygon) 
{
  gsl_matrix * line = gsl_matrix_alloc(polygon->size1, polygon->size2 + 1);

  for (size_t i = 0; i < polygon->size2; i++) {
    gsl_vector_view p = gsl_matrix_column(polygon, i);
    gsl_matrix_set_col(line, i, &p.vector); 
  }

  gsl_vector_view start = gsl_matrix_column(polygon, 0);
  gsl_matrix_set_col(line, polygon->size2, &start.vector); 
  return line;
}



gsl_vector* math2d_line_equation(gsl_vector* pt1, gsl_vector* pt2){
  gsl_vector* line = gsl_vector_alloc(2);
  double m = math2d_slope(pt1, pt2);
  double b = gsl_vector_get(pt1, 1) - gsl_vector_get(pt1, 0) * m;
  
  gsl_vector_set(line, 0, m);
  gsl_vector_set(line, 1, b);
  return line;
}

/**
 * Find the intersection point of two lines defined as segments.
 * The intersection point might not be on the two segments. 
 * Implemented following
 * http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
 * @bound  True if it should only return intersection points on the segment.
 * Returns:  the intersectino point, or NULL if there wasn't one. 
*/
gsl_vector* math2d_intersect_segments(gsl_vector* pt1_seg1, gsl_vector* pt2_seg1, 
                                      gsl_vector* pt1_seg2, gsl_vector* pt2_seg2, bool bound) {
  double x1 = gsl_vector_get(pt1_seg1, 0); double y1 = gsl_vector_get(pt1_seg1, 1);
  double x2 = gsl_vector_get(pt2_seg1, 0); double y2 = gsl_vector_get(pt2_seg1, 1);
  double x3 = gsl_vector_get(pt1_seg2, 0); double y3 = gsl_vector_get(pt1_seg2, 1);
  double x4 = gsl_vector_get(pt2_seg2, 0); double y4 = gsl_vector_get(pt2_seg2, 1);

  double denom = ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));
  if(denom == 0){
    return NULL;
  }

  double ua = (((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3))) / denom;
  double ix = x1 + ua*(x2 - x1);
  double iy = y1 + ua*(y2 - y1);
  
  gsl_vector* ret_vec = gsl_vector_alloc(2);
  gsl_vector_set(ret_vec, 0, ix);
  gsl_vector_set(ret_vec, 1, iy);
  
  //bool math2d_is_on_segment(gsl_vector* seg_st_xy, gsl_vector* seg_end_xy, gsl_vector* p_xy){
  if(!bound || (math2d_is_on_segment(pt1_seg1, pt2_seg1, ret_vec) 
		&& math2d_is_on_segment(pt1_seg2, pt2_seg2, ret_vec)))
    return ret_vec;
  else{
    gsl_vector_free(ret_vec);
    return NULL;
  }
}

gsl_vector* math2d_intersect_line_segment_line(gsl_vector* pt1_seg, gsl_vector* pt2_seg, double m, double b){
  gsl_vector* pt1_seg2 = gsl_vector_alloc(2);
  gsl_vector_set(pt1_seg2, 0, 0);
  gsl_vector_set(pt1_seg2, 1, b);
  
  gsl_vector* pt2_seg2 = gsl_vector_alloc(2);
  gsl_vector_set(pt2_seg2, 0, 1);
  gsl_vector_set(pt2_seg2, 1, m*1+b);

  //[(0, b), (1, m*1+b)], false);
  //gsl_vector* ret_vec = math2d_intersect_segments(pt1_seg, pt2_seg, pt1_seg2, pt2_seg2, false);

  gsl_vector* ret_vec = math2d_intersect_segments(pt1_seg, pt2_seg, pt1_seg2, pt2_seg2, false);

  gsl_vector_free(pt1_seg2);
  gsl_vector_free(pt2_seg2);
  
  
  if(ret_vec == NULL)
    return NULL;
  else if(!math2d_is_on_segment(pt1_seg, pt2_seg, ret_vec)){
    gsl_vector_free(ret_vec);
    return NULL;
  }
  
  return ret_vec;
}


gsl_matrix* math2d_intersect_polygon_line_analytic(gsl_matrix* polygon, double m, double b) {
  gsl_matrix* tmp_pts = gsl_matrix_alloc(polygon->size1, polygon->size2);
  int counter = 0;
  
  //p1, p2 in zip(line, line[1:]):					   
  for(size_t i=0; i<polygon->size2; i++){
    gsl_vector_view p1;  gsl_vector_view p2;
    if(i == polygon->size2-1){
      p1 = gsl_matrix_column(polygon, polygon->size2-1);
      p2 = gsl_matrix_column(polygon, 0);
    }
    else{
      p1 = gsl_matrix_column(polygon, i);
      p2 = gsl_matrix_column(polygon, i+1);
    }
    
    gsl_vector* p = math2d_intersect_line_segment_line(&p1.vector, &p2.vector, m, b); 
    
    /*check to make sure that the point hasn't already been added*/
    bool add_point = true;
    if(p!= NULL && counter > 0){
      gsl_vector* tmp_dist = tklib_get_distance(&gsl_matrix_submatrix(tmp_pts, 0, 0, polygon->size1, counter).matrix, p);
      if(gsl_vector_min(tmp_dist) < 10e-5)
	add_point = false;
      gsl_vector_free(tmp_dist);
    }
    
    if(p != NULL && add_point){ // && math2d_is_on_segment(&p1.vector, &p2.vector, p)){
      //results.add(p);
      gsl_matrix_set_col(tmp_pts, counter, p);
      counter +=1;
      gsl_vector_free(p);
    }
    else if(p!=NULL)
      gsl_vector_free(p);
  }

  if(counter == 0){
    gsl_matrix_free(tmp_pts);
    return NULL;
  }
  
  gsl_matrix* ret_pts = gsl_matrix_alloc(polygon->size1, counter);
  gsl_matrix_memcpy(ret_pts, &gsl_matrix_submatrix(tmp_pts, 0, 0, polygon->size1, counter).matrix);
  
  gsl_matrix_free(tmp_pts);
  return ret_pts;
}


gsl_matrix * math2d_intersect_polygon_line(gsl_matrix * polygon_xy, gsl_matrix * line_xy)
{  
  gsl_matrix * pline = math2d_polygon_to_line(polygon_xy);
  
  gsl_matrix * results = math2d_intersect_lines(line_xy, pline);
  
  gsl_matrix_free(pline);
  return results;
}

gsl_matrix * math2d_intersect_lines(gsl_matrix * line1_xy, gsl_matrix * line2_xy)
{
  gsl_matrix* tmp_results = gsl_matrix_alloc(line1_xy->size1, 
					     line1_xy->size2 * line2_xy->size2);

  gsl_vector* last = gsl_vector_calloc(line1_xy->size1);
  int counter = 0;

  for (size_t i = 0; i < line1_xy->size2 - 1; i++) {
      gsl_vector_view p_1; gsl_vector_view p_2;
      p_1 = gsl_matrix_column(line1_xy, i);
      p_2 = gsl_matrix_column(line1_xy, i + 1);
      for (size_t j = 0; j < line2_xy->size2 - 1; j++) {
        gsl_vector_view m_1; gsl_vector_view m_2;
        m_1 = gsl_matrix_column(line2_xy, j);
        m_2 = gsl_matrix_column(line2_xy, j + 1);        
        
        gsl_vector* p = math2d_intersect_segments(&p_1.vector, &p_2.vector, 
                                                  &m_1.vector, &m_2.vector, true);
        if (p != NULL && 
            math2d_is_on_segment(&p_1.vector, &p_2.vector, p) && 
            math2d_is_on_segment(&m_1.vector, &m_2.vector, p)) {
          if (counter == 0 || !math2d_point_equal(p, last)) {
            gsl_matrix_set_col(tmp_results, counter, p);
            gsl_vector_memcpy(last, p);
            counter +=1;
          }
        }
        gsl_vector_free(p);
      }
  }
  
  gsl_vector_free(last);

  if (counter == 0) {
    gsl_matrix_free(tmp_results);
    return NULL;
  }

  //copy the relevant parts
  gsl_matrix* results = gsl_matrix_alloc(tmp_results->size1, counter);
  gsl_matrix_view tmp_res = gsl_matrix_submatrix(tmp_results, 0, 0, tmp_results->size1, counter);
  gsl_matrix_memcpy(results, &tmp_res.matrix);
  gsl_matrix_free(tmp_results);
  
  return results;

}

/* This function intersects two polygons (assumed to be open) */
gsl_matrix* math2d_intersect_polygon_polygon(gsl_matrix* polygon1_xy, gsl_matrix* polygon2_xy){
  gsl_matrix* tmp_results = gsl_matrix_alloc(polygon1_xy->size1, 
					     polygon1_xy->size2 * 
                                             polygon2_xy->size2 );
  gsl_vector* last = gsl_vector_calloc(polygon1_xy->size1);
  gsl_vector_set_all(last, -10e-100);
  int counter = 0;
  
  for(size_t i=0; i< polygon1_xy->size2; i++){
    for(size_t j=0; j<polygon2_xy->size2; j++){

      /*get the first segment*/
      gsl_vector_view p1_1; gsl_vector_view p1_2;
      if(i == polygon1_xy->size2-1){
	p1_1 = gsl_matrix_column(polygon1_xy, polygon1_xy->size2-1);
	p1_2 = gsl_matrix_column(polygon1_xy, 0);
      }
      else{
	p1_1 = gsl_matrix_column(polygon1_xy, i);
	p1_2 = gsl_matrix_column(polygon1_xy, i+1);
      }

      /*Get the second segment*/
      gsl_vector_view p2_1; gsl_vector_view p2_2;
      if(j == polygon2_xy->size2-1){
	p2_1 = gsl_matrix_column(polygon2_xy, polygon2_xy->size2-1);
	p2_2 = gsl_matrix_column(polygon2_xy, 0);
      }
      else{
	p2_1 = gsl_matrix_column(polygon2_xy, j);
	p2_2 = gsl_matrix_column(polygon2_xy, j+1);
      }
      
      //gsl_vector* math2d_intersect_segment(gsl_vector* pt1_seg1, gsl_vector* pt2_seg1, 
      //gsl_vector* pt1_seg2, gsl_vector* pt2_seg2, bool bound){

      gsl_vector* p = math2d_intersect_segments(&p1_1.vector, &p1_2.vector, 
                                                &p2_1.vector, &p2_2.vector, true);
      
      //&& isOnSegment([p1, p2], p) and   isOnSegment([m1, m2], p))
      if (p != NULL && math2d_dist(p, last) > 0){
	  //&& math2d_is_on_segment(&p1_1.vector, &p1_2.vector, p) 
	  //&& math2d_is_on_segment(&p2_1.vector, &p2_2.vector, p)){
	gsl_matrix_set_col(tmp_results, counter, p);
	gsl_vector_memcpy(last, p);
	counter +=1;
      }
      
      if(p != NULL)
	gsl_vector_free(p);	
    }
  }
  
  gsl_vector_free(last);

  //if there was no intersection, then return NULL
  if(counter == 0){
    gsl_matrix_free(tmp_results);
    return NULL;
  }


  //copy the relevant parts
  gsl_matrix* results = gsl_matrix_alloc(tmp_results->size1, counter);
  gsl_matrix_view tmp_res = gsl_matrix_submatrix(tmp_results, 0, 0, tmp_results->size1, counter);
  gsl_matrix_memcpy(results, &tmp_res.matrix);
  
  gsl_matrix_free(tmp_results);

  
  return results;
}

/**
 * Computed following this web page: 
 * http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
 */
double math2d_signed_area(gsl_matrix * polygon)
{
  double area = 0;
  for (size_t i = 0; i < polygon->size2; i++) {
    size_t j = (i + 1) % polygon->size2;
    double xi = gsl_matrix_get(polygon, 0, i);
    double yi = gsl_matrix_get(polygon, 1, i);
    double xj = gsl_matrix_get(polygon, 0, j);
    double yj = gsl_matrix_get(polygon, 1, j);
    area += (xi * yj - xj * yi);
  }
  area = 0.5 * area;
  return area;
}

double math2d_area(gsl_matrix * polygon)
{
  return fabs(math2d_signed_area(polygon));
}


/**
 * Determine if a point is inside a polygon
 */
bool math2d_is_interior_point(gsl_vector* pt, gsl_matrix* polygon_xy){
  double x = gsl_vector_get(pt, 0);
  double y = gsl_vector_get(pt, 1);
  bool c = false;
  for (size_t i = 0; i < polygon_xy->size2; i++) {
    int j = (i + 1) % polygon_xy->size2;
    gsl_vector_view p1 = gsl_matrix_column(polygon_xy, i);
    gsl_vector_view p2 = gsl_matrix_column(polygon_xy, j);


    double x1 = gsl_vector_get(&p1.vector, 0);
    double y1 = gsl_vector_get(&p1.vector, 1);

    double x2 = gsl_vector_get(&p2.vector, 0);
    double y2 = gsl_vector_get(&p2.vector, 1);

    if (x == x1 && y == y1) {
      return true;
    }

    if ((((y1 <= y) && (y < y2)) ||
         ((y2 <= y) && (y < y1))) &&
        (x < ((x2 - x1) * (y - y1)) / (y2 - y1) + x1)) {
      c = !c;
    }

  }
  return c;
}


//if(startPoint == None)
//startPoint = midpoint(segment);

gsl_matrix* math2d_perpendicular_segment(gsl_vector* seg_st, gsl_vector* seg_end, gsl_vector* start_point){
  //(x1, y1), (x2, y2) = segment;
  //startX, startY = startPoint;
  double segmentLength = math2d_dist(seg_st, seg_end);
  gsl_matrix* ret_mat = gsl_matrix_alloc(seg_st->size, 2);
  
  //isDegenerate(segment))
  /* if its degenerate, return the same segment */
  if(segmentLength <= 10e-5){
    gsl_matrix_set_col(ret_mat, 0, seg_st);
    gsl_matrix_set_col(ret_mat, 1, seg_end);
  }
  //isVertical(segment))
  /* if its vertical return something else */
  else if(fabs(gsl_vector_get(seg_st, 0) - gsl_vector_get(seg_end, 0)) < 10e-5){
    //return [(x1 - segmentLength/2.0, startY), (x1 + segmentLength/2.0, startY)];
    gsl_matrix_set(ret_mat, 0, 0, gsl_vector_get(seg_st, 0) - segmentLength/2.0);
    gsl_matrix_set(ret_mat, 1, 0, gsl_vector_get(start_point, 1));
    
    gsl_matrix_set(ret_mat, 0, 1, gsl_vector_get(seg_st, 0) + segmentLength/2.0);
    gsl_matrix_set(ret_mat, 1, 1, gsl_vector_get(start_point, 1));
  }
  else{
    //m, b = lineEquation(segment);
    gsl_vector* line = math2d_line_equation(seg_st, seg_end);
    double m = gsl_vector_get(line, 0);
    
    if(fabs(m) > 10e-5){
      //      return [(newX1, newM * newX1 + newB),(newX2, newM * newX2 + newB)];
      double newM = -1.0/m;
      double newB = gsl_vector_get(start_point, 1) - gsl_vector_get(start_point, 0)*newM;
      double offset = fabs(gsl_vector_get(seg_end, 0) - gsl_vector_get(seg_st, 0))/2.0;
      double newX1 = gsl_vector_get(start_point, 0) - offset;
      double newX2 = gsl_vector_get(start_point, 0) + offset;

      gsl_matrix_set(ret_mat, 0, 0, newX1);
      gsl_matrix_set(ret_mat, 1, 0, newM*newX1+newB);
      gsl_matrix_set(ret_mat, 0, 1, newX2);
      gsl_matrix_set(ret_mat, 1, 1, newM*newX2+newB);
    }

    else{
      //return [(startX, y1 - segmentLength/2.0), (startX, y1 + segmentLength/2.0)];
      gsl_matrix_set(ret_mat, 0, 0, gsl_vector_get(start_point, 0));
      gsl_matrix_set(ret_mat, 1, 0, gsl_vector_get(seg_st, 1) - segmentLength/2.0);
      gsl_matrix_set(ret_mat, 0, 1, gsl_vector_get(start_point, 0));
      gsl_matrix_set(ret_mat, 1, 1, gsl_vector_get(seg_st, 1) + segmentLength/2.0);
    }

    gsl_vector_free(line);
  }
  
  return ret_mat;
}

double math2d_dist_along_segment(gsl_vector * seg_st_xy, gsl_vector * seg_end_xy, gsl_vector * pt) 
{
  assert(math2d_is_on_segment(seg_st_xy, seg_end_xy, pt));
  return math2d_dist(seg_st_xy, pt);
}


    
double math2d_dist_along_line(gsl_matrix * l_xy, gsl_vector * p_xy)
{
  double dist_along_line = 0.0;
  for (size_t i = 0; i < l_xy->size2 - 1; i++) {
    gsl_vector_view seg_st_xy = gsl_matrix_column(l_xy, i);
    gsl_vector_view seg_end_xy = gsl_matrix_column(l_xy, i+1);
    if (math2d_is_on_segment(&seg_st_xy.vector, &seg_end_xy.vector, p_xy)) {
      return dist_along_line + math2d_dist(&seg_st_xy.vector, p_xy);
    }
    dist_along_line += math2d_dist(&seg_st_xy.vector, &seg_end_xy.vector);
  }
  printf("Point not on line\n");
  tklib_vector_printf(p_xy);
  tklib_matrix_printf(l_xy);
  assert(0);
}
 
/**
 * Returns a subset of the line that starts at p1 and ends at p2.
 * p1 and p2 have to be on the line. 
 * Returns the line itself if p1 and p2 aren't on the line. 
 */
gsl_matrix * math2d_trim_line(gsl_matrix * line_xy, gsl_vector * p1, gsl_vector * p2) 
{
  
  if (!math2d_is_on_line(line_xy, p1) || !math2d_is_on_line(line_xy, p2)) {
    return math2d_matrix_copy(line_xy);
  }
  gsl_vector * start_point;
  gsl_vector * end_point;

  if (math2d_dist_along_line(line_xy, p1) < math2d_dist_along_line(line_xy, p2)) {
    start_point = p1;
    end_point = p2;
  } else {
    start_point = p2;
    end_point = p1;
  }
  gsl_matrix* trimmed_line_xy = gsl_matrix_alloc(line_xy->size1, line_xy->size2);
  bool is_before = true;
  int trimmed_line_idx = 0;
  for (size_t i = 0; i < line_xy->size2 - 1; i++) {
    gsl_vector_view p1_xy = gsl_matrix_column(line_xy, i);
    gsl_vector_view p2_xy = gsl_matrix_column(line_xy, i+1);
    if (is_before && math2d_is_on_segment(&p1_xy.vector, &p2_xy.vector, start_point)) {
      gsl_matrix_set_col(trimmed_line_xy, trimmed_line_idx, start_point);
      trimmed_line_idx++;
      is_before = false;
    }
    if (math2d_is_on_segment(&p1_xy.vector, &p2_xy.vector, end_point)) {
      gsl_matrix_set_col(trimmed_line_xy, trimmed_line_idx, end_point);
      trimmed_line_idx++;
      
      gsl_matrix * result = math2d_copy_point_list_up_to(trimmed_line_xy, 
trimmed_line_idx);
      gsl_matrix_free(trimmed_line_xy);
      return result;
    }
    if (! is_before) {
      gsl_matrix_set_col(trimmed_line_xy, trimmed_line_idx, &p2_xy.vector);
      trimmed_line_idx++;
    }
  }
  assert(0);
}

/**
 * Returns a subsequence of points from the polygon, starting at p1, and
 * ending at p2.  p1 and p2 do not have to be vertices, but all other
 * points in the list will be vertices of the polygon.
 * p1 and p2 have to be on the polygon. 
*/  
gsl_matrix* math2d_trim_polygon(gsl_matrix * polygon, gsl_vector * p1, gsl_vector * p2) 
{
  gsl_vector_view polygon_end = gsl_matrix_column(polygon, polygon->size2 - 1);
  gsl_matrix * l1 = math2d_trim_line(polygon, p1,  &polygon_end.vector); 
  
  
  gsl_matrix * l2 = math2d_point_list_append(l1, polygon);

  gsl_matrix * result = math2d_trim_line(l2, p1, p2);

  gsl_matrix_free(l2);
  gsl_matrix_free(l1);
  return result;
}

double math2d_line_length(gsl_matrix * line_xy) 
{
  double dist_along_line = 0;
  for (size_t i = 0; i < line_xy->size2-1; i++) {
    gsl_vector_view p1 = gsl_matrix_column(line_xy, i);
    gsl_vector_view p2 = gsl_matrix_column(line_xy, i+1);
    dist_along_line += math2d_dist(&p1.vector, &p2.vector);
  }
  return dist_along_line;
}

double math2d_point_length(gsl_vector * point) 
{
  double x = gsl_vector_get(point, 0);
  double y = gsl_vector_get(point, 1);
  return pow(pow(x, 2) + pow(y, 2), 0.5);
}


gsl_matrix* math2d_compute_boundary_line(gsl_matrix* landmark_xy, gsl_matrix* figure_xy) 
{

  gsl_vector_view start_f =  gsl_matrix_column(figure_xy, 0);
  gsl_vector_view end_f =  gsl_matrix_column(figure_xy, figure_xy->size2-1);

  gsl_vector * start_g = math2d_closest_point_on_line(landmark_xy, &start_f.vector);
  gsl_vector * end_g = math2d_closest_point_on_line(landmark_xy, &end_f.vector);
    
  gsl_matrix * l1 = math2d_trim_polygon(landmark_xy, start_g, end_g);
  gsl_matrix * l2 = math2d_trim_polygon(landmark_xy, end_g, start_g);

  gsl_matrix * retval;
  if (math2d_line_length(l1) <= math2d_line_length(l2)) {
    retval = l1;
    gsl_matrix_free(l2);
  } else {
    retval = l2;
    gsl_matrix_free(l1);
  }
  gsl_vector_free(start_g);
  gsl_vector_free(end_g);

  return retval;
}

/**
 * Computes the axes the figure imposes on the landmark.
 * Returns them as a matrix
 */
struct axes math2d_compute_axes(gsl_matrix* ground_xy, gsl_matrix* figure_xy){
  //bad example, no axes
  if (figure_xy == NULL || ground_xy == NULL) {
    return math2d_empty_axes();
  }


  //endPointSegment = [figureGeom[0], figureGeom[-1]];
  gsl_vector_view fig_st = gsl_matrix_column(figure_xy, 0);
  gsl_vector_view fig_end = gsl_matrix_column(figure_xy, figure_xy->size2-1);

  //if (math2d_double_equal(gsl_vector_get(&fig_st.vector, 0), 44.16493099)) {
  // verbose = 1;
  //}
  gsl_matrix * intersect_points = math2d_intersect_polygon_line(ground_xy, figure_xy);

  if (intersect_points == NULL && !math2d_is_interior_point(&fig_st.vector, ground_xy)) {
    gsl_matrix_free(intersect_points);
    //printf("math2d: returning empty axes\n");
    return math2d_empty_axes();
  }

  
  gsl_matrix* gnd_combined = math2d_combined_matrix(ground_xy, &gsl_matrix_submatrix(ground_xy, 0, 
										    0, ground_xy->size1, 1).matrix);

  //math2d.isDegenerate(endPointSegment))
  //not math2d.isVertical(endPointSegment) 

  gsl_matrix* intersectPoints; 
  // if the two points aren't vertical and they aren't at the same location
  if (gsl_vector_get(&fig_st.vector, 0) == gsl_vector_get(&fig_end.vector, 0)
      || math2d_dist(&fig_st.vector, &fig_end.vector) <= 10e-5) {
    gsl_vector_view Y = gsl_matrix_row(ground_xy, 1);
    double y_max = gsl_vector_max(&Y.vector);
    double y_min = gsl_vector_min(&Y.vector);

    /* move around the start-end points around a bit. */
    gsl_matrix* munged_fig_axis = gsl_matrix_alloc(2, 2);
    gsl_matrix_set(munged_fig_axis, 0, 0, gsl_vector_get(&fig_st.vector, 0));
    gsl_matrix_set(munged_fig_axis, 1, 0, y_min-10e10);
    gsl_matrix_set(munged_fig_axis, 0, 1, gsl_vector_get(&fig_st.vector, 0));
    gsl_matrix_set(munged_fig_axis, 1, 1, y_max+10e10);
    
    //math2d.polygonToLine(groundGeom),munged_fig_axis);
    intersectPoints = math2d_intersect_polygon_polygon(ground_xy, munged_fig_axis);
    gsl_matrix_free(munged_fig_axis);
  } else {
    
    gsl_vector* line = math2d_line_equation(&fig_st.vector, &fig_end.vector);

    

    intersectPoints = math2d_intersect_polygon_line_analytic(ground_xy, 
                                                             gsl_vector_get(line, 0), 
                                                             gsl_vector_get(line, 1));

    gsl_vector_free(line);
  }

  
  struct axes result;
  /* if there are no intersection points, check and see if one is contained in the other */
  if(intersectPoints == NULL){
    //bool math2d_is_interior_point(gsl_vector* pt, gsl_matrix* mpolygon){

    /*set the minor axis according to nearest neighbor */
    if(math2d_is_interior_point(&fig_st.vector, ground_xy)){
      result.minor_st = math2d_closest_point_on_line(gnd_combined, &fig_st.vector);
      result.minor_end = math2d_closest_point_on_line(gnd_combined, &fig_end.vector);
    }
    else {
      result = math2d_empty_axes();
    }
  }
  /* if there is a single intersection point, check and see if one is contained in the other */
  /* get the start and end points of the intersection */
  else if(intersectPoints->size2 == 1 
	  || math2d_dist(&gsl_matrix_column(intersectPoints, 0).vector, 
				      &gsl_matrix_column(intersectPoints, intersectPoints->size2-1).vector) <= 10e-5){

    if(math2d_is_interior_point(&fig_st.vector, ground_xy)) {
      result.minor_st = math2d_closest_point_on_line(gnd_combined, &fig_st.vector);
    } else {
      result.minor_st = gsl_vector_alloc(intersectPoints->size1);
      gsl_matrix_get_col(result.minor_st, intersectPoints, 0);
    }
    result.minor_end = math2d_closest_point_on_line(gnd_combined, &fig_end.vector);
  }
  
  /* I think this is wrong, ... you basically want to get the 
     intersection points that are as far apart as possible ... 
     especially for the features that get computed */
  else {
    gsl_vector* D = tklib_get_distance(intersectPoints, origin);
    
    size_t min_i; size_t max_i;
    gsl_vector_minmax_index(D, &min_i, &max_i);
    
    result.minor_st = math2d_vector_copy(&gsl_matrix_column(intersectPoints, min_i).vector);
    result.minor_end = math2d_vector_copy(&gsl_matrix_column(intersectPoints, max_i).vector);
    
    gsl_vector_free(D);
  }
  
  
  gsl_matrix_free(gnd_combined);
  
  if(intersectPoints != NULL)
    gsl_matrix_free(intersectPoints);

  if (result.minor_st == NULL) {
    return result;
  }
  
  gsl_vector * minor_midpoint = math2d_point((gsl_vector_get(result.minor_st, 0) + 
                                              gsl_vector_get(result.minor_end, 0)) * 0.5,
                                             (gsl_vector_get(result.minor_st, 1) + 
                                              gsl_vector_get(result.minor_end, 1)) * 0.50);
  gsl_matrix* major = math2d_perpendicular_segment(result.minor_st, result.minor_end, 
                                                   minor_midpoint);


  /* set the major axis */
  gsl_vector_view major_st = gsl_matrix_column(major, 0);
  gsl_vector_view major_end = gsl_matrix_column(major, 1);
  result.major_st = math2d_vector_copy(&major_st.vector);
  result.major_end = math2d_vector_copy(&major_end.vector);
  
  /* free the memory */
  gsl_matrix_free(major);
  gsl_vector_free(minor_midpoint);
  
  return result;
}

double math2d_dist_between_points_along_line(gsl_matrix * line, gsl_vector * p1, gsl_vector * p2)
{
  return math2d_dist_along_line(line, p2) - math2d_dist_along_line(line, p1);
}

double math2d_dist_between_points_along_polygon(gsl_matrix * polygon, gsl_vector * p1, gsl_vector * p2)
{
  double per = math2d_perimeter(polygon);
  gsl_matrix * line = math2d_polygon_to_line(polygon);
  double s1 = fabs(math2d_dist_between_points_along_line(line, p1, p2));
  double s2 = per - s1;
  gsl_matrix_free(line);
  if (s1 < s2) {
    return s1;
  } else {
    return s2;
  }
}
double math2d_perimeter(gsl_matrix * polygon_xy)
{
  gsl_vector_view start = gsl_matrix_column(polygon_xy, 0);
  gsl_vector_view end = gsl_matrix_column(polygon_xy, polygon_xy->size2 - 1);
  
  return math2d_dist(&start.vector, &end.vector) + math2d_line_length(polygon_xy);
}


void math2d_free_eigenstuff(struct eigenstuff in) 
{
  gsl_vector_free(in.evals);
  gsl_matrix_free(in.evecs);
}

/**
 * Compute the axes that correspond to the eigen vectors of the polygon.
 * For a long rectangular table, these axes will correspond to the "natural" 
 * coordinate system of the table.
 */

struct axes math2d_eigen_axes(gsl_matrix * polygon)
{
  struct eigenstuff estuff = math2d_eigenvectors(polygon);
  gsl_vector * a0 = math2d_vector_copy(&(gsl_matrix_column(estuff.evecs, 0)).vector);
  gsl_vector * a1 = math2d_vector_copy(&(gsl_matrix_column(estuff.evecs, 1)).vector);

  double u_0 = gsl_vector_get(estuff.evals, 0);
  double u_1 = gsl_vector_get(estuff.evals, 1);

  gsl_vector_scale(a0, u_0);
  gsl_vector_scale(a1, u_1);



  gsl_vector * major;
  gsl_vector * minor;
  if (u_0 > u_1) {
    major = a0;
    minor = a1;
  } else {
    major = a1;
    minor = a0;
  }
  gsl_vector * centroid = math2d_centroid(polygon);      
  struct axes result;

  result.major_st = math2d_vector_copy(centroid);
  gsl_vector_sub(result.major_st, major);

  result.major_end = math2d_vector_copy(centroid);
  gsl_vector_add(result.major_end, major);

  result.minor_st = math2d_vector_copy(centroid);
  gsl_vector_sub(result.minor_st, minor);
    
  result.minor_end = math2d_vector_copy(centroid);
  gsl_vector_add(result.minor_end, minor);


  gsl_vector_free(centroid);
  gsl_vector_free(a0);
  gsl_vector_free(a1);
  math2d_free_eigenstuff(estuff);
  return result;
}


/**
 * Following levin02principal.pdf, which is in doc/analytical-eigen-axes/
 * Idea for eigen axes still from Blissard, but the algorithm is from levin.
 */
struct eigenstuff math2d_eigenvectors(gsl_matrix * polygon)
{
  int n = polygon->size1;
  int d = polygon->size2;
  assert (n == 2);
  gsl_matrix * I = tklib_eye(d, d);
  gsl_matrix * E = tklib_ones(d, d);
  gsl_matrix_add(E, I);
  
  gsl_matrix * A = polygon;
  gsl_matrix * At = tklib_transpose(A);
  gsl_matrix * AdotE = tklib_matrix_dot(A, E);
  
  gsl_matrix * final = tklib_matrix_dot(AdotE, At);

  double scale = 1.0 / (d * (d + 1));
  gsl_matrix_scale(final, scale);

  gsl_vector *evals = gsl_vector_alloc (final->size1);
  gsl_matrix *evecs = gsl_matrix_alloc (final->size1, final->size2);
     
  gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc (final->size1);
       
  gsl_eigen_symmv (final, evals, evecs, w);
     
  gsl_eigen_symmv_free (w);
     
  gsl_eigen_symmv_sort (evals, evecs, 
                        GSL_EIGEN_SORT_ABS_DESC);

  eigenstuff result;
  result.evals = evals;
  result.evecs = evecs;

  gsl_matrix_free(I);
  gsl_matrix_free(E);
  gsl_matrix_free(At);
  gsl_matrix_free(AdotE);
  gsl_matrix_free(final);

  return result;
}




/**
 *Computed following this web page: 
 *http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
 */
gsl_vector * math2d_centroid(gsl_matrix * polygon)
{
  double cx = 0.0;
  double cy = 0.0;
  double a = math2d_signed_area(polygon);
  if  (a == 0) {
    gsl_vector_view X = gsl_matrix_row(polygon, 0);
    gsl_vector_view Y = gsl_matrix_row(polygon, 1);
    return math2d_point(tklib_vector_mean(&X.vector), 
                        tklib_vector_mean(&Y.vector));
  }
  for (size_t i = 0; i < polygon->size2; i++) {
    size_t j = (i + 1) % polygon->size2;
    double xi = gsl_matrix_get(polygon, 0, i);
    double yi = gsl_matrix_get(polygon, 1, i);
    double xj = gsl_matrix_get(polygon, 0, j);
    double yj = gsl_matrix_get(polygon, 1, j);
    double multiplier = xi * yj - xj * yi;
    cx += multiplier * (xi + xj);
    cy += multiplier * (yi + yj);
  }
  cx = cx / (6.0*a);
  cy = cy / (6.0*a);
  return math2d_point(cx, cy);
}


struct fit_line_result math2d_fit_line(gsl_matrix * points)
{
  
  gsl_vector * x_values = &(gsl_matrix_row(points, 0).vector);
  gsl_vector * y_values = &(gsl_matrix_row(points, 1).vector);

  double cov00;
  double cov01; 
  double cov11;
  double sumsq;
  fit_line_result result;
  gsl_fit_linear(x_values->data, x_values->stride, 
                 y_values->data, y_values->stride,
                 points->size2,
                 &result.intercept, &result.slope, 
                 &cov00, &cov01, &cov11, &sumsq);

  return result;
 }


gsl_vector * math2d_center_of_mass(gsl_matrix * points)
{
  double x_total = 0;
  double y_total = 0;
  
  for (size_t i = 0; i < points->size2; i++) {
    double xi = gsl_matrix_get(points, 0, i);
    double yi = gsl_matrix_get(points, 1, i);
    x_total += xi;
    y_total += yi;
  }
  return math2d_point(x_total/points->size2, 
                      y_total/points->size2);
}


void math2d_points_printf(gsl_matrix * points) 
{
  if (points == NULL) {
    printf("NULL\n");
  } else {
    printf("[");
    for (size_t i = 0; i < points->size2; i++) {
      double xi = gsl_matrix_get(points, 0, i);
      double yi = gsl_matrix_get(points, 1, i);
      printf("(%.8f, %.8f),", xi, yi);
    }
    printf("]\n");
  }
}


void math2d_vector_printf(gsl_vector * points) 
{
  printf("[");
  for (size_t i = 0; i < points->size; i++) {
    double val = gsl_vector_get(points, i);
    printf("%.12f, ",  val);
  }
  printf("]\n");
}

struct math2d_range math2d_smallest_window(gsl_vector * lst, int window_size)
{
  double window_total = math2d_vector_sum(lst, 0, window_size);
  double min_window = window_total;
  struct math2d_range max_range;
  max_range.start_i = 0;
  max_range.end_i = window_size;
  
  struct math2d_range current_range;
  current_range.start_i = 0;
  current_range.end_i = window_size;
  
  for (size_t i = 1; i < lst->size - window_size + 1;  i++) {
    int start_idx = i;
    int end_idx = start_idx + window_size - 1;
    window_total -= gsl_vector_get(lst, start_idx - 1);
    window_total += gsl_vector_get(lst, end_idx);
    if (window_total < min_window) {
      min_window = window_total;
      max_range.start_i = start_idx;
      max_range.end_i = end_idx + 1;
    }
  }

  return max_range;
}


double math2d_vector_sum(gsl_vector* vec, int start_idx, int end_idx)
{
  double sum=0;
  int i;
  for(i = start_idx; i < end_idx; i++){
    sum += gsl_vector_get(vec, i);
  }
  return sum;
}

/*
 * Returns the "top" of the polygon, where up is defined as
 * direction_xy, following regier01.  The part of the polygon that
 * gets wet when rain is coming from direction_xy. 
 *
 * direction_xy points "up", or in the opposite direction that the
 * rain is moving.
 *
 */
gsl_matrix * math2d_top(gsl_matrix * polygon, gsl_vector * direction_xy)
{
  
  gsl_matrix * result = gsl_matrix_alloc(polygon->size1, polygon->size2);
  gsl_vector * dir_lots_xy = math2d_vector_to_unit_vector(direction_xy);
  gsl_vector_scale(dir_lots_xy, math2d_perimeter(polygon));

  gsl_vector * p_plus_lots = math2d_point(0, 0);

  int idx = 0;
  for (size_t i = 0; i < polygon->size2; i++) {
    gsl_vector * p = &(gsl_matrix_column(polygon, i).vector);

    gsl_vector_memcpy(p_plus_lots, p);
    gsl_vector_add(p_plus_lots, dir_lots_xy);
    if (math2d_is_visible(polygon, p, p_plus_lots)) {
      gsl_matrix_set_col(result, idx, p);
      idx++;
    }
  }
  
  gsl_matrix * final_result;
  if(idx == 0){
    gsl_vector * p = &(gsl_matrix_column(polygon, 0).vector);
    gsl_matrix_set_col(result, idx, p); idx++;
    final_result = math2d_copy_point_list_up_to(result, idx);
  }
  else
    final_result = math2d_copy_point_list_up_to(result, idx);


  gsl_matrix_free(result);
  gsl_vector_free(p_plus_lots);
  gsl_vector_free(dir_lots_xy);
  return final_result;
}

double math2d_height_in_direction(gsl_vector * point, 
                                  gsl_vector * direction)
{
  gsl_vector * closest_p = math2d_closest_point_on_segment_line(origin, 
                                                                direction,
                                                                point);
  double dist =  math2d_dist(origin, closest_p);
  double dot = tklib_vector_dot(closest_p, direction);
  if (dot < 0) {
    dist = dist * -1;
  }
  gsl_vector_free(closest_p);
  return dist;
}


/**
 * Returns the point that is farthest in direction_xy.
 */
gsl_vector * math2d_highest_point(gsl_matrix * points, 
                                  gsl_vector * direction_xy)
{
  double max_dist = GSL_NEGINF;

  int max_idx = -1;
  for (size_t i = 0; i < points->size2; i++) {
    gsl_vector * p = &(gsl_matrix_column(points, i).vector);
    double dist =  math2d_height_in_direction(p, direction_xy);

    if (dist > max_dist) {
      max_dist = dist;
      max_idx = i;
    }
  }
  return math2d_vector_copy(&(gsl_matrix_column(points, max_idx).vector));
}

/**
 * Returns the point that is least in direction_xy.
 */
gsl_vector * math2d_lowest_point(gsl_matrix * points, 
                                 gsl_vector * direction_xy)
{
  gsl_vector * opposite_dir = math2d_vector_copy(direction_xy);
  gsl_vector_scale(opposite_dir, -1);
  gsl_vector * result = math2d_highest_point(points, direction_xy);
  gsl_vector_free(opposite_dir);
  return result;
}


/**
 * Returns whether p1 can see p2, if polygon is the only obstacle in the world.
 */
bool math2d_is_visible(gsl_matrix * polygon, gsl_vector * p1, gsl_vector * p2)
{

  if (math2d_is_on_polygon(polygon, p1) &&
      math2d_is_on_polygon(polygon, p2)) {
    return false;
  }
  for (size_t i = 0; i < polygon->size2; i++) {
    gsl_vector * p = &(gsl_matrix_column(polygon, i).vector);
    gsl_vector * next_p = &(gsl_matrix_column(polygon, 
                                               (i + 1) % polygon->size2).vector);
    gsl_vector * intersect_pt =  math2d_intersect_segments(p, next_p,
                                                           p1, p2, true);
    if (intersect_pt != NULL) {
      if (math2d_point_equal(intersect_pt, p1) &&
          (! math2d_is_on_segment(p1, p2, next_p) ||
           ! math2d_is_on_segment(p1, p2, p))) {
        gsl_vector_free(intersect_pt);
        continue;
      } else if (math2d_point_equal(intersect_pt, p2) &&
          (! math2d_is_on_segment(p1, p2, next_p) ||
           ! math2d_is_on_segment(p1, p2, p))) {
        gsl_vector_free(intersect_pt);
        continue;
      } else {
        gsl_vector_free(intersect_pt);
        return false;
      }
      gsl_vector_free(intersect_pt);
    }
  }
  return true;
}

gsl_vector * math2d_vector_to_unit_vector(gsl_vector * direction_xy)
{
  double x = gsl_vector_get(direction_xy, 0);
  double y = gsl_vector_get(direction_xy, 1);
  double length = pow(pow(x, 2) + pow(y, 2), 0.5);
  return math2d_point(x/length, y/length);
}

bool math2d_range_overlaps(double r1_s, double r1_e, double r2_s, double r2_e) 
{
  return (math2d_between(r1_s, r2_s, r2_e) ||
          math2d_between(r1_e, r2_s, r2_e) ||
          math2d_between(r2_s, r1_s, r1_e) ||
          math2d_between(r2_e, r1_s, r1_e));
}

bool math2d_overlaps(gsl_matrix * polygon1, gsl_matrix * polygon2)
{
  gsl_matrix * intersect_points = math2d_intersect_polygon_polygon(polygon1, 
                                                                    polygon2);
  gsl_vector_view p1 = gsl_matrix_column(polygon1, 0);
  gsl_vector_view p2 = gsl_matrix_column(polygon2, 0);
  bool result;
  if (math2d_is_interior_point(&p1.vector, polygon2) ||
      math2d_is_interior_point(&p2.vector, polygon1)) {
    result = true;
  } else {
    if (intersect_points != NULL) {
      result = true;
    } else {
      result = false;
    }
  }
  
  if(intersect_points != NULL)
    gsl_matrix_free(intersect_points);
  return result;
}

gsl_vector * math2d_midpoint_segment(gsl_vector * p1, gsl_vector * p2)
{
  gsl_vector * midpoint = math2d_point((gsl_vector_get(p1, 0) + 
                                        gsl_vector_get(p2, 0)) * 0.5,
                                       (gsl_vector_get(p1, 1) + 
                                        gsl_vector_get(p2, 1)) * 0.50);
  return midpoint;
}

int math2d_cmp(double v1, double v2) 
{
  if (v1 < v2) {
    return -1;
  } else if (v1 > v2) {
    return 1;
  } else {
    assert(v1 == v2);
    return 0;
  }
}

/**
 * Compare two points, returning 0 if equal, -1 if p1 < p2, 1 if p2 > p2.
 * They are sorted lexically, first by X and then by Y. 
 */
int math2d_cmp_points(const gsl_vector * p1, const gsl_vector * p2)
{
  double x1 = gsl_vector_get(p1, 0);
  double y1 = gsl_vector_get(p1, 1);

  double x2 = gsl_vector_get(p2, 0);
  double y2 = gsl_vector_get(p2, 1);
  
  int cmp_x1_x2 = math2d_cmp(x1, x2);
  int cmp_y1_y2 = math2d_cmp(y1, y2);

  if (cmp_x1_x2 != 0) {
    return cmp_x1_x2;
  } else {
    if (cmp_y1_y2 != 0) {
      return cmp_y1_y2;
    }
  }
  assert(x1 == x2);
  assert(y1 == y2);
  return 0;
}

/**
 * private void * version to pass to the sort function.
 */
int _math2d_cmp_points(const void * a1, const void * a2)
{
  const gsl_vector_view * p1 = ((gsl_vector_view *) a1);
  const gsl_vector_view * p2 = ((gsl_vector_view *) a2);
  return math2d_cmp_points(&p1->vector, &p2->vector);
}

gsl_matrix * math2d_sort_points(gsl_matrix * points)
{
  gsl_vector_view * array = (gsl_vector_view *) malloc(sizeof(gsl_vector_view) * points->size2);
  for (size_t i = 0; i < points->size2; i ++) {
    array[i] = gsl_matrix_column(points, i);
  }

  qsort(array, points->size2, sizeof(gsl_vector *), _math2d_cmp_points);
  
  gsl_matrix * result = gsl_matrix_alloc(points->size1, points->size2);
  for (size_t i = 0; i < points->size2; i ++) {
    gsl_matrix_set_col(result, i, &array[i].vector);
  }
  free(array);
  return result;
}

double math2d_angle(gsl_vector * vector)
{
  double x = gsl_vector_get(vector, 0);
  double y = gsl_vector_get(vector, 1);
  //double r = math2d_point_length(vector);  
  //return acos(x/r);
  return atan2(y, x);
}
gsl_vector * math2d_rotate(gsl_vector * vector,
                           double theta)
{
  double r = math2d_point_length(vector);
  double newtheta = theta + math2d_angle(vector);
  return math2d_point(r * cos(newtheta), r * sin(newtheta));

}


