#ifndef MATH3D_H
#define MATH3D_H

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_math.h>
#include <assert.h>
#include "gsl_utilities.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
  gsl_matrix * points_xy;
  double z_start;
  double z_end;
  int initialized; // should change all args to use pointers, because this doesn't play nice with swig.
} math3d_prism_t;

math3d_prism_t math3d_prism_init();
void math3d_prism_free(math3d_prism_t p);
bool math3d_higher_than(math3d_prism_t p1, math3d_prism_t p2);
bool math3d_starts_higher_than(math3d_prism_t p1, math3d_prism_t p2); 
bool math3d_supports(math3d_prism_t p1, math3d_prism_t p2);
bool math3d_intersect_prisms(math3d_prism_t p1, math3d_prism_t p2);
gsl_matrix * math3d_compress(gsl_vector * timestamps,
                             gsl_matrix * points_xyztheta);

double math3d_dist(gsl_vector* pt1, gsl_vector * pt2);
double math3d_square_dist(gsl_vector* pt1, gsl_vector * pt2);


gsl_vector * math3d_intersect_line_plane(gsl_matrix * line_xyz, 
                                         gsl_matrix * plane_xyz);


#ifdef __cplusplus
}
#endif


#endif

