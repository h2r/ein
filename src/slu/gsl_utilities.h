#ifndef GSL_UTIL1_H
#define GSL_UTIL1_H

#ifdef __cplusplus
extern "C" {
#endif
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_vector_short.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
//#include <marray/marray.h>
#include <math.h>

//gsl_matrix* tklib_marray_2matrix(marray* t);
//gsl_vector* tklib_marray_get_row(marray* myarray, int dim, size_t* index);

gsl_vector* tklib_vector_leq(gsl_vector* vec, double value);
gsl_vector* tklib_vector_geq(gsl_vector* vec, double value);


gsl_matrix* tklib_inverse(gsl_matrix* M);

gsl_vector* tklib_double_to_gsl_vector(double* myarray, int length);
gsl_matrix* tklib_rtheta_to_xy(gsl_vector* pose, gsl_vector* reading);
gsl_matrix* tklib_rtheta_to_xy_matrix(gsl_vector* pose, gsl_matrix* RTh);

gsl_matrix* tklib_xy_to_rtheta(gsl_vector* curr_pose, gsl_matrix* features);

gsl_matrix* tklib_matrix_get(gsl_matrix* M, gsl_vector* Ir, gsl_vector* Ic);
gsl_vector* tklib_matrix_get_vector(gsl_matrix* M, gsl_vector* Ir, gsl_vector* Ic);
gsl_matrix* tklib_matrix_get_columns(gsl_matrix* M, gsl_vector* I);
gsl_matrix* tklib_matrix_get_rows(gsl_matrix* M, gsl_vector* I);

gsl_vector* tklib_vector_get(gsl_vector* V, gsl_vector* I);

gsl_matrix_float* tklib_matrix_float_get_columns(gsl_matrix_float* M, gsl_vector* I);
gsl_matrix_float* tklib_matrix_float_get_rows(gsl_matrix_float* M, gsl_vector* I);


gsl_vector* tklib_vector_log(gsl_vector* vec);
gsl_vector* tklib_vector_exp(gsl_vector* vec);

gsl_matrix* tklib_exp(gsl_matrix* mat);
gsl_matrix_float* tklib_log_float(gsl_matrix_float* mat);

gsl_matrix* tklib_log(gsl_matrix* mat);
gsl_matrix_float* tklib_exp_float(gsl_matrix_float* mat);

gsl_vector* tklib_sin(gsl_vector *angles);
gsl_vector* tklib_cos(gsl_vector *angles);
gsl_vector* tklib_arctan2(gsl_vector *Y, gsl_vector* X);

gsl_vector *tklib_range(double start, double end, double increment);
gsl_matrix* tklib_linalg_cholesky_decomp(gsl_matrix* A);
double tklib_linalg_det(gsl_matrix* A);

double tklib_sse(gsl_matrix* mat, gsl_matrix* opts);
double tklib_trace(gsl_matrix* mat);
gsl_matrix* tklib_eye(int n1, int n2);
gsl_matrix* tklib_subtract_mean(gsl_matrix* pts);



inline double tklib_euclidean_distance(gsl_vector* pt1, gsl_vector* pt2){
  double dot_val;
  gsl_vector* p1_minus_p2 = gsl_vector_calloc(pt1->size);
  gsl_vector_memcpy(p1_minus_p2, pt1);

  //p1-p2
  gsl_vector_sub(p1_minus_p2, pt2);
  
  //take the dot product of the distances with itself
  gsl_blas_ddot(p1_minus_p2, p1_minus_p2, &dot_val);
  
  double dist = sqrt(dot_val);
  gsl_vector_free(p1_minus_p2);
  return dist;
}


gsl_vector* tklib_get_distance(gsl_matrix* X, gsl_vector* pt);
gsl_vector* tklib_permutation_to_vector(gsl_permutation* permutation);

gsl_matrix* tklib_diag(gsl_vector* vec);
gsl_vector* tklib_mean(gsl_matrix *pts, int dimension);
gsl_vector* tklib_get_centroid(gsl_vector* x_st, gsl_vector* x_end);

gsl_vector* tklib_matrix_sum(gsl_matrix* A, int dimension);
gsl_vector* tklib_matrix_prod(gsl_matrix* A, int dimension);
double tklib_vector_sum(gsl_vector* vec);
double tklib_vector_prod(gsl_vector* vec);

void tklib_vector_sqrt(gsl_vector* vec);
void tklib_matrix_sqrt(gsl_matrix* X);

gsl_vector* tklib_matrix_argmin(gsl_matrix* A, int dimension);
gsl_vector* tklib_matrix_min(gsl_matrix* A, int dimension);

void tklib_matrix_printf(gsl_matrix* themat);
void tklib_matrix_float_printf(gsl_matrix_float* themat);
void tklib_vector_printf(gsl_vector* thevec);
void tklib_permutation_printf(gsl_permutation* thevec);
void tklib_const_matrix_printf(const gsl_matrix* themat);


gsl_matrix* tklib_matrix_mul_vec(gsl_matrix* pts, gsl_vector *vec);
void tklib_matrix_mul_vec_inplace(gsl_matrix* pts, gsl_vector *vec);

void tklib_matrix_add_vec(gsl_matrix* pts, gsl_vector *vec, double alpha, double beta);
void tklib_matrix_add_vec_M(gsl_matrix* pts, gsl_matrix *vec, double alpha, double beta);

void tklib_vector_union(gsl_vector * mask1, gsl_vector * mask2, gsl_vector * dest);
void tklib_vector_intersect(gsl_vector * mask1, gsl_vector * mask2, gsl_vector * dest);
void tklib_apply_mask_lp(gsl_vector * mask, gsl_vector * log_probs, gsl_vector * dest); 

extern inline double tklib_normalize_theta(double theta)
{
  int multiplier;
  
  if (theta >= -M_PI && theta < M_PI)
    return theta;
  
  multiplier = (int)(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}

gsl_vector* tklib_normalize_theta_array(gsl_vector* theta);
gsl_matrix * tklib_ones(int d1, int d2);
gsl_matrix * tklib_transpose(gsl_matrix * m);
gsl_matrix * tklib_matrix_dot(gsl_matrix * m1, gsl_matrix *m2);
double tklib_vector_dot(gsl_vector * v1, gsl_vector * v2);

int tklib_vector_argmin(gsl_vector * vector) ;
int tklib_vector_argmax(gsl_vector * vector) ;

double tklib_vector_mean(gsl_vector * vector) ;
double tklib_vector_stddev(gsl_vector * vector) ;
double tklib_vector_variance(gsl_vector * vector) ;

double tklib_vector_equal(gsl_vector * v1, gsl_vector * g2);

gsl_vector * tklib_vector_linspace(double start, double stop, int num_units);

int tklib_vector_bisect(gsl_vector * v, double value);

#ifdef __cplusplus
}
#endif

#endif


