#include "math3d.h"
#include "math2d.h"

void math3d_set_point(gsl_vector * ret, double x, double y, double z)
{
  gsl_vector_set(ret, 0, x);
  gsl_vector_set(ret, 1, y);
  gsl_vector_set(ret, 2, z);
}


gsl_vector * math3d_point(double x, double y, double z)
{
  gsl_vector * result = gsl_vector_alloc(3);
  math3d_set_point(result, x, y, z);
  return result;
}

void math3d_prism_free(math3d_prism_t p)
{
  gsl_matrix_free(p.points_xy);
}

math3d_prism_t math3d_prism_init()
{
  math3d_prism_t result;
  result.points_xy = 0;
  result.initialized = 1;
  return result;
}


bool math3d_higher_than(math3d_prism_t p1, math3d_prism_t p2) 
{
  if (p1.z_end > p2.z_end) {
    return true;
  } else {
    return false;
  }
}

bool math3d_starts_higher_than(math3d_prism_t p1, math3d_prism_t p2) 
{
  if (p1.z_start > p2.z_start) {
    return true;
  } else {
    return false;
  }
}

bool math3d_supports(math3d_prism_t p1, math3d_prism_t p2)
{
  if (math3d_starts_higher_than(p2, p1) &&
      math2d_overlaps(p1.points_xy, p2.points_xy)) {
    return true;
  } else {
    return false;
  }
}



bool math3d_intersect_prisms(math3d_prism_t p1, math3d_prism_t p2)
{
  if (math2d_range_overlaps(p1.z_start, p1.z_end, p2.z_start, p2.z_end) && 
      math2d_overlaps(p1.points_xy, p2.points_xy)) {
    return true;
  } else {
    return false;
  }
}

/**
 * Returns a matrix fo txyztheta, the timestamps and points,
 * compressed so that if there are lots of timestamps with the same
 * point, there is only one.
 */
gsl_matrix * math3d_compress(gsl_vector * timestamps,
                             gsl_matrix * points_xyztheta)
{
  if (timestamps == NULL || points_xyztheta == NULL) {
    return NULL;
  }
  assert(points_xyztheta->size2 == timestamps->size);
  gsl_matrix * matrix = gsl_matrix_alloc(points_xyztheta->size1 + 1, 
                                         points_xyztheta->size2);
  gsl_matrix * tmp_matrix = &(gsl_matrix_submatrix(matrix, 0, 0, points_xyztheta->size1,
                                                   points_xyztheta->size2).matrix);

  gsl_vector * tmp_timestamps = &(gsl_matrix_row(matrix, points_xyztheta->size1).vector);
  
  int current_idx = 0;
  for (size_t i = 0; i < timestamps->size; i++) {
    bool append = false;
    gsl_vector * this_point = &gsl_matrix_column(points_xyztheta, i).vector;

    if (current_idx == 0) {
      append = true;
    } else {
      gsl_vector * last_point = &gsl_matrix_column(tmp_matrix, 
                                                   current_idx - 1).vector;

      if (! tklib_vector_equal(last_point, this_point)) {
        append = true;
      }
    }
    if (append) {
      gsl_vector_set(tmp_timestamps, 
                     current_idx, gsl_vector_get(timestamps, i));
      gsl_matrix_set_col(tmp_matrix, current_idx, this_point);
      current_idx += 1;
    }
  }  

  gsl_matrix * real_result = math2d_copy_point_list_up_to(matrix, current_idx);
  gsl_matrix_free(matrix);
  return real_result; 
}
double math3d_square_dist(gsl_vector* pt1, gsl_vector * pt2) 
{
  double x1 = gsl_vector_get(pt1, 0);
  double y1 = gsl_vector_get(pt1, 1);
  double z1 = gsl_vector_get(pt1, 2);
  
  double x2 = gsl_vector_get(pt2, 0);
  double y2 = gsl_vector_get(pt2, 1);
  double z2 = gsl_vector_get(pt2, 2);
  
  double xdiff = x2 - x1;
  double ydiff = y2 - y1;
  double zdiff = z2 - z1;

  return xdiff * xdiff + ydiff * ydiff + zdiff * zdiff;
}

double math3d_dist(gsl_vector* pt1, gsl_vector * pt2)
{
  return sqrt(math3d_square_dist(pt1, pt2));
}

/**
 * The intersection of a line (defined by two 3d points)
 * and a plane (defined by three 3d points).
 */
gsl_vector * math3d_intersect_line_plane(gsl_matrix * line_xyz, 
                                         gsl_matrix * plane_xyz)
{

  
  gsl_vector_view la = gsl_matrix_column(line_xyz, 0);
  gsl_vector_view lb = gsl_matrix_column(line_xyz, 1);


  gsl_vector_view p0 = gsl_matrix_column(plane_xyz, 0);
  gsl_vector_view p1 = gsl_matrix_column(plane_xyz, 1);
  gsl_vector_view p2 = gsl_matrix_column(plane_xyz, 2);


  gsl_vector * u = math3d_point(0, 0, 0);
  gsl_vector_add(u, &la.vector);
  gsl_vector_sub(u, &p0.vector);

  

  gsl_matrix * m = gsl_matrix_alloc(3, 3);

  gsl_vector_view m0 = gsl_matrix_column(m, 0);
  gsl_vector_view m1 = gsl_matrix_column(m, 1);
  gsl_vector_view m2 = gsl_matrix_column(m, 2);

  gsl_vector_memcpy(&m0.vector, &la.vector);
  gsl_vector_sub(&m0.vector, &lb.vector);

  gsl_vector_memcpy(&m1.vector, &p1.vector);
  gsl_vector_sub(&m1.vector, &p0.vector);

  gsl_vector_memcpy(&m2.vector, &p2.vector);
  gsl_vector_sub(&m2.vector, &p0.vector);

  int signum;
  gsl_permutation* p = gsl_permutation_calloc(3);
  gsl_linalg_LU_decomp(m, p, &signum);

  double m_det = gsl_linalg_LU_det(m, signum);

  gsl_vector * result = NULL;
  if (m_det != 0) {
    gsl_matrix * m_inv = gsl_matrix_alloc(3, 3);


    result = math3d_point(0, 0, 0);
    gsl_linalg_LU_invert(m, p, m_inv);
    
    gsl_vector * tuv = math3d_point(0, 0, 0);
    
    gsl_blas_dgemv (CblasNoTrans, 1.0, m_inv,
                    u, 1.0, tuv);
    
    
    gsl_vector_memcpy(result, &lb.vector);
    gsl_vector_sub(result, &la.vector);
    gsl_vector_scale(result, gsl_vector_get(tuv, 0));
    gsl_vector_add(result, &la.vector);
    gsl_vector_free(tuv);
    gsl_matrix_free(m_inv);

  }
  gsl_permutation_free(p);    
  gsl_matrix_free(m);
  gsl_vector_free(u);

  return result;
}
