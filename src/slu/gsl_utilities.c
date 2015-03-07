#include "gsl_utilities.h"
#include <gsl/gsl_math.h>
#include <assert.h>

gsl_matrix* tklib_inverse(gsl_matrix* M){
  int signum;
  gsl_matrix* LU_cov = gsl_matrix_calloc(M->size1, M->size2);
  gsl_matrix* LU_inv = gsl_matrix_calloc(M->size2, M->size1);
  gsl_matrix_memcpy(LU_cov, M);
  
  gsl_permutation* p_cov = gsl_permutation_calloc(M->size2);
  gsl_linalg_LU_decomp(LU_cov, p_cov, &signum);
  
  //perform the inverse covariance
  gsl_linalg_LU_invert(LU_cov, p_cov, LU_inv);
  
  gsl_matrix_free(LU_cov);
  gsl_permutation_free(p_cov);
  
  return LU_inv;
}

gsl_matrix_float* tklib_exp_float(gsl_matrix_float* mat){
  size_t i, j;
  gsl_matrix_float* mymat = gsl_matrix_float_alloc(mat->size1, mat->size2);
  
  for(i=0;i<mat->size1;i++){
    for(j=0;j<mat->size2;j++){
      gsl_matrix_float_set(mymat, i, j, exp(gsl_matrix_float_get(mat, i,j)));
    }
  }
  
  return mymat;
}

gsl_vector* tklib_vector_leq(gsl_vector* vec, double value){
  gsl_vector* myvec = gsl_vector_alloc(vec->size);

  size_t i;  
  for(i=0;i<vec->size;i++){
    gsl_vector_set(myvec, i, gsl_vector_get(vec, i) <= value);
  }
  
  return myvec;
}


gsl_vector* tklib_vector_geq(gsl_vector* vec, double value){
  gsl_vector* myvec = gsl_vector_alloc(vec->size);

  size_t i;  
  for(i=0;i<vec->size;i++){
    gsl_vector_set(myvec, i, gsl_vector_get(vec, i) >= value);
  }
  
  return myvec;
}


gsl_matrix* tklib_exp(gsl_matrix* mat){
  size_t i, j;
  gsl_matrix* mymat = gsl_matrix_alloc(mat->size1, mat->size2);
  
  for(i=0;i<mat->size1;i++){
    for(j=0;j<mat->size2;j++){
      gsl_matrix_set(mymat, i, j, exp(gsl_matrix_get(mat, i,j)));
    }
  }
  
  return mymat;
}


gsl_vector* tklib_vector_exp(gsl_vector* vec){
  gsl_vector* myvec = gsl_vector_alloc(vec->size);

  size_t i;  
  for(i=0;i<vec->size;i++){
    gsl_vector_set(myvec, i, exp(gsl_vector_get(vec, i)));
  }
  
  return myvec;
}

gsl_vector* tklib_vector_log(gsl_vector* vec){
  gsl_vector* myvec = gsl_vector_alloc(vec->size);

  size_t i;  
  for(i=0;i<vec->size;i++){
    gsl_vector_set(myvec, i, log(gsl_vector_get(vec, i)));
  }
  
  return myvec;
}



gsl_matrix_float* tklib_log_float(gsl_matrix_float* mat){
  size_t i, j;
  gsl_matrix_float* mymat = gsl_matrix_float_alloc(mat->size1, mat->size2);
  
  for(i=0;i<mat->size1;i++){
    for(j=0;j<mat->size2;j++){
      gsl_matrix_float_set(mymat, i, j, log(gsl_matrix_float_get(mat, i,j)));
    }
  }
  
  return mymat;
}


gsl_matrix* tklib_log(gsl_matrix* mat){
  size_t i, j;
  gsl_matrix* mymat = gsl_matrix_alloc(mat->size1, mat->size2);
  
  for(i=0;i<mat->size1;i++){
    for(j=0;j<mat->size2;j++){
      gsl_matrix_set(mymat, i, j, log(gsl_matrix_get(mat, i,j)));
    }
  }
  
  return mymat;
}




gsl_matrix* tklib_rtheta_to_xy(gsl_vector* pose, gsl_vector* reading){
  double r_x = gsl_vector_get(pose, 0);  
  double r_y = gsl_vector_get(pose, 1);
  double r_th = gsl_vector_get(pose, 2);
  
  gsl_matrix* XY = gsl_matrix_calloc(2, reading->size);

  gsl_vector* theta = tklib_range(-M_PI/2.0, M_PI/2.0+(M_PI/180.0), M_PI/180.0);
  //gsl_vector* theta = tklib_range(-M_PI/2.0, M_PI/2.0+0.001, M_PI/180.0);
  gsl_vector_add_constant(theta, r_th);
  
  //X = reading*cos(pose.theta + theta)+pose.x;
  gsl_vector* X = tklib_cos(theta);
  gsl_vector_mul(X, reading);
  gsl_vector_add_constant(X, r_x);

  //Y = reading*sin(pose.theta + theta)+pose.y;
  gsl_vector* Y = tklib_sin(theta);
  gsl_vector_mul(Y, reading);
  gsl_vector_add_constant(Y, r_y);  
  
  //set the values
  gsl_matrix_set_row(XY, 0, X);
  gsl_matrix_set_row(XY, 1, Y);
  
  //free the extra memory
  gsl_vector_free(theta);
  gsl_vector_free(X);
  gsl_vector_free(Y);
  return XY;
}

gsl_matrix* tklib_rtheta_to_xy_matrix(gsl_vector* pose, gsl_matrix* RTh){
  double r_x = gsl_vector_get(pose, 0);  
  double r_y = gsl_vector_get(pose, 1);
  double r_th = gsl_vector_get(pose, 2);
  
  gsl_matrix* XY = gsl_matrix_calloc(2, RTh->size2);

  //get R and theta
  gsl_vector_view R = gsl_matrix_row(RTh, 0);
  gsl_vector_view Th = gsl_matrix_row(RTh, 1);
  
  gsl_vector* theta = gsl_vector_calloc((&Th.vector)->size);
  gsl_vector_memcpy(theta, &Th.vector);

  gsl_vector_add_constant(theta, r_th);
  
  //X = reading*cos(pose.theta + theta)+pose.x;
  gsl_vector* X = tklib_cos(theta);
  gsl_vector_mul(X, &R.vector);
  gsl_vector_add_constant(X, r_x);

  //Y = reading*sin(pose.theta + theta)+pose.y;
  gsl_vector* Y = tklib_sin(theta);
  gsl_vector_mul(Y, &R.vector);
  gsl_vector_add_constant(Y, r_y);  
  
  //set the values
  gsl_matrix_set_row(XY, 0, X);
  gsl_matrix_set_row(XY, 1, Y);
  
  //free the extra memory
  gsl_vector_free(X);
  gsl_vector_free(Y);
  gsl_vector_free(theta);
  return XY;
}


gsl_matrix* tklib_xy_to_rtheta(gsl_vector* curr_pose, gsl_matrix* features){
  gsl_matrix* ret_features = gsl_matrix_calloc(features->size1, features->size2);
  

  //get the location and the theta of the point
  gsl_vector_view curr_pt = gsl_vector_subvector(curr_pose, 0, 2);
  double theta = gsl_vector_get(curr_pose, 2);
  
  //copy into working memory
  gsl_matrix* features_tmp = gsl_matrix_alloc(features->size1, features->size2);
  gsl_matrix_memcpy(features_tmp, features);
    
  
  //subtract the current locaiton from the features
  //curr_ref_frame = features - array([[curr_pose.x], [curr_pose.y]]);
  tklib_matrix_add_vec(features_tmp, &curr_pt.vector, 1.0, -1.0);
  
  gsl_vector_view x_pts = gsl_matrix_row(features_tmp, 0);
  gsl_vector_view y_pts = gsl_matrix_row(features_tmp, 1);
  
  //compute the arctan of these values to get the global angle and then
  //   add the offset for the robot position
  gsl_vector* Phi = tklib_arctan2(&y_pts.vector, &x_pts.vector);

  gsl_vector_add_constant(Phi, -theta);
  gsl_vector *Phi_pr = tklib_normalize_theta_array(Phi);
  
  //get the distances from the current pose to the points
  gsl_vector* zeros = gsl_vector_calloc(2);
  gsl_vector* D = tklib_get_distance(features_tmp, zeros);

  gsl_matrix_set_row(ret_features, 0, D);
  gsl_matrix_set_row(ret_features, 1, Phi_pr);
  
  gsl_vector_free(Phi);
  gsl_vector_free(Phi_pr);
  gsl_vector_free(D);
  gsl_vector_free(zeros);
  gsl_matrix_free(features_tmp);
  
  //#normalize this
  //Phi = normalize_theta_array(arctan2(curr_ref_frame[1,:], curr_ref_frame[0,:])-curr_pose.theta);
  //D  = sqrt(curr_ref_frame[0,:]**2 + curr_ref_frame[1,:]**2);
  //return D, Phi;

  return ret_features;
}



gsl_vector* tklib_normalize_theta_array(gsl_vector* theta){
  if(theta == NULL)
    return NULL;

  size_t i;

  gsl_vector* ret_thetas = gsl_vector_alloc(theta->size);
  for(i=0;i<theta->size;i++)
    gsl_vector_set(ret_thetas, i, tklib_normalize_theta(gsl_vector_get(theta, i)));

  return ret_thetas;
}

gsl_vector* tklib_double_to_gsl_vector(double* myarray, int length){
  gsl_block* bl = (gsl_block*)calloc(1, sizeof(gsl_block));
  bl->size = length;
  bl->data = myarray;
  
  gsl_vector* mydata = gsl_vector_alloc_from_block(bl, 0, length, 1);

  return mydata;
}


gsl_matrix* tklib_matrix_get(gsl_matrix* M, gsl_vector* Ir, gsl_vector* Ic){
  gsl_matrix* retmat = gsl_matrix_alloc(Ir->size, Ic->size);
  size_t i;
  for(i=0;i<Ic->size;i++){
    gsl_vector_view v = gsl_matrix_column(M, (int)gsl_vector_get(Ic,i));
    gsl_vector* v_new = tklib_vector_get(&v.vector, Ir);
    
    gsl_matrix_set_col(retmat, i, v_new);
    gsl_vector_free(v_new);
  }
  
  return retmat;
}


gsl_vector* tklib_matrix_get_vector(gsl_matrix* M, gsl_vector* Ir, gsl_vector* Ic){
  gsl_vector* retvec = gsl_vector_alloc(Ir->size);
  size_t i;
  for(i=0;i<Ic->size;i++){
    double res = gsl_matrix_get(M, gsl_vector_get(Ir, i), gsl_vector_get(Ic, i));
    gsl_vector_set(retvec, i, res);
  }
  
  return retvec;
}


gsl_matrix* tklib_matrix_get_columns(gsl_matrix* M, gsl_vector* I){
  gsl_matrix* retmat = gsl_matrix_alloc(M->size1, I->size);
  size_t i;
  for(i=0;i<I->size;i++){
    gsl_vector_view v = gsl_matrix_column(M, (int)gsl_vector_get(I,i));
    gsl_matrix_set_col(retmat, i, &v.vector);
  }
  
  return retmat;
}

gsl_matrix* tklib_matrix_get_rows(gsl_matrix* M, gsl_vector* I){
  gsl_matrix* retmat = gsl_matrix_alloc(I->size, M->size2);
  size_t i;
  for(i=0;i<I->size;i++){
    gsl_vector_view v = gsl_matrix_row(M, (int)gsl_vector_get(I,i));
    gsl_matrix_set_row(retmat, i, &v.vector);
  }
  
  return retmat;
}

gsl_vector* tklib_vector_get(gsl_vector* V, gsl_vector* I){
  gsl_vector* retvec = gsl_vector_alloc(I->size);
  size_t i;
  for(i=0;i<I->size;i++){
    double v = gsl_vector_get(V, (int)gsl_vector_get(I,i));
    gsl_vector_set(retvec, i, v);
  }
  
  return retvec;
}



gsl_matrix_float* tklib_matrix_float_get_columns(gsl_matrix_float* M, gsl_vector* I){
  gsl_matrix_float* retmat = gsl_matrix_float_alloc(M->size1, I->size);
  size_t i;
  for(i=0;i<I->size;i++){
    gsl_vector_float_view v = gsl_matrix_float_column(M, (int)gsl_vector_get(I,i));
    gsl_matrix_float_set_col(retmat, i, &v.vector);
  }
  
  return retmat;
}


gsl_matrix_float* tklib_matrix_float_get_rows(gsl_matrix_float* M, gsl_vector* I){
  gsl_matrix_float* retmat = gsl_matrix_float_alloc(I->size, M->size2);
  size_t i;
  for(i=0;i<I->size;i++){
    gsl_vector_float_view v = gsl_matrix_float_row(M, (int)gsl_vector_get(I,i));
    gsl_matrix_float_set_row(retmat, i, &v.vector);
  }
  
  return retmat;
}

gsl_vector* tklib_sin(gsl_vector *angles){
  gsl_vector* thesin = gsl_vector_alloc(angles->size);

  size_t i;
  for(i=0;i<angles->size;i++){
    gsl_vector_set(thesin, i, sin(gsl_vector_get(angles, i)));
  }

  return thesin;
}

gsl_vector* tklib_arctan2(gsl_vector *Y, gsl_vector* X){
  gsl_vector* theatan2 = gsl_vector_alloc(Y->size);

  size_t i;
  double x, y;
  for(i=0;i<Y->size;i++){
    y = gsl_vector_get(Y, i);
    x = gsl_vector_get(X, i);
    gsl_vector_set(theatan2, i, atan2(y, x));
  }
  
  return theatan2;
}

gsl_vector* tklib_cos(gsl_vector *angles){
  gsl_vector* thecos = gsl_vector_alloc(angles->size);

  size_t i;
  for(i=0;i<angles->size;i++){
    gsl_vector_set(thecos, i, cos(gsl_vector_get(angles, i)));
  }

  return thecos;
}


double tklib_linalg_det(gsl_matrix* A){
  gsl_matrix* Apr = gsl_matrix_alloc(A->size1, A->size2);
  gsl_matrix_memcpy(Apr, A);

  gsl_permutation* p = gsl_permutation_calloc(A->size2);
  
  int signum;
  gsl_linalg_LU_decomp(Apr, p, &signum);
  double mydet = gsl_linalg_LU_det(Apr, signum);
  
  gsl_matrix_free(Apr);
  gsl_permutation_free(p);
  return mydet;
}

gsl_vector *tklib_range(double start, double end, double increment){
  int num_elts = (int)(((end-start)/increment)+1);

  gsl_vector* ret_vec = gsl_vector_alloc(num_elts);
  
  size_t i;
  for(i=0;(int)i<num_elts;i++){
    gsl_vector_set(ret_vec, i, start+i*increment);
  }

  return ret_vec;
}


gsl_matrix* tklib_linalg_cholesky_decomp(gsl_matrix* A){
  gsl_matrix* L = gsl_matrix_alloc(A->size1, A->size2);
  gsl_matrix_memcpy(L, A);
  
  gsl_linalg_cholesky_decomp(L);
  
  size_t i, j;
  for(i=0;i<L->size1;i++){
    for(j=0;j<L->size2;j++){
      if(j>i){
	gsl_matrix_set(L, i, j, 0.0);
      }
    }
  }
  
  return L;
}

gsl_vector* tklib_get_centroid(gsl_vector* x_st, gsl_vector* x_end){
  
  // intended code
  //  midpoint = self.x_st + ((self.x_end - self.x_st)/2.0);
  gsl_vector* midpoint = gsl_vector_calloc(x_st->size);
  gsl_vector_memcpy(midpoint, x_end);
  gsl_vector_sub(midpoint, x_st);
  gsl_vector_scale(midpoint, 1.0/2.0);
  gsl_vector_add(midpoint, x_st);

  return midpoint;
}



gsl_vector* tklib_get_distance(gsl_matrix* X, gsl_vector* pt){
  gsl_matrix* dist_mat = gsl_matrix_calloc(X->size1, X->size2);
  gsl_matrix_memcpy(dist_mat, X);
  
  //get the distances here
  tklib_matrix_add_vec(dist_mat, pt, 1.0, -1.0);
  gsl_matrix_mul_elements(dist_mat, dist_mat);
  
  gsl_vector* dist_i = tklib_matrix_sum(dist_mat, 0);
  tklib_vector_sqrt(dist_i);
  //free the allocated variables
  gsl_matrix_free(dist_mat);
  
  return dist_i;
}

gsl_vector* tklib_permutation_to_vector(gsl_permutation* permutation){
  size_t mysize = gsl_permutation_size(permutation);
  gsl_vector* retperm = gsl_vector_calloc(mysize);

  size_t i;
  size_t* myperm = gsl_permutation_data(permutation);
  for(i=0;i<mysize;i++){
    gsl_vector_set(retperm, i, myperm[i]);
  }
  
  return retperm;
}

gsl_matrix* tklib_diag(gsl_vector* vec){
  gsl_matrix* mat = gsl_matrix_calloc(vec->size, vec->size);
  
  size_t i;
  for(i=0;i<vec->size;i++){
    gsl_matrix_set(mat, i, i, gsl_vector_get(vec, i));
  }
  
  return mat;
}

gsl_vector* tklib_mean(gsl_matrix *pts, int dimension){
  gsl_vector* mean_vector;

  //dimenstion 0 means that we perform the mean over the rows
  if(dimension == 0){
    mean_vector = tklib_matrix_sum(pts, 1); 
    gsl_vector_scale(mean_vector, 1.0/pts->size2);
  }

  //dimension 1 means that we perform the mean over the columns
  else{
    mean_vector = tklib_matrix_sum(pts, 0); 
    gsl_vector_scale(mean_vector, 1.0/pts->size1);
  }

  return mean_vector;
}


double tklib_sse(gsl_matrix* mat, gsl_matrix* opts){
  //allocate a second matrix here
  gsl_matrix* sub_pts = gsl_matrix_calloc(mat->size1, mat->size2);
  
  //copy the points into another matrix
  gsl_matrix_memcpy(sub_pts, mat);

  //subtract the other points from the current ones
  gsl_matrix_sub(sub_pts, opts);
  
  //get the square of the error
  gsl_matrix_mul_elements(sub_pts, sub_pts);
  //printf("mul_elements\n");
  //tklib_matrix_printf(sub_pts);
  //sum the error up
  //printf("sum_matrix\n");
  //tklib_vector_printf(sum_matrix(sub_pts, 0));
  
  gsl_vector* vec_sum = tklib_matrix_sum(sub_pts, 1);
  double sse = tklib_vector_sum(vec_sum);
  
  gsl_matrix_free(sub_pts);
  gsl_vector_free(vec_sum);
  return sse;
}

double tklib_trace(gsl_matrix* mat){
  double result=0;
  size_t i;
  for(i = 0; i < mat->size1; i++){
    size_t j;
    for(j = 0; j < mat->size2; j++){
      result += gsl_matrix_get(mat, i, j);
    }
  }

  return result;
}


gsl_matrix* tklib_eye(int n1, int n2){
  gsl_matrix* theeye = gsl_matrix_calloc(n1, n2);

  size_t i;
  for(i = 0; i < theeye->size1; i++){
    gsl_matrix_set(theeye, i, i, 1.0);
  }

  return theeye;
}


gsl_matrix* tklib_subtract_mean(gsl_matrix* pts){
  gsl_matrix* pts_norm = gsl_matrix_calloc(pts->size1, pts->size2);
  gsl_matrix_memcpy(pts_norm, pts);
  gsl_vector* pts_mean_vec = tklib_mean(pts, 0);
  tklib_matrix_add_vec(pts_norm, pts_mean_vec, 1.0, -1.0);

  gsl_vector_free(pts_mean_vec);
  return pts_norm;
}

double tklib_vector_sum(gsl_vector* vec){
  double sum=0;
  size_t i;
  for(i = 0; i < vec->size; i++){
    sum += gsl_vector_get(vec, i);
  }
  return sum;
}

double tklib_vector_prod(gsl_vector* vec){
  double myprod=1.0;
  size_t i;
  for(i = 0; i < vec->size; i++){
    myprod *= gsl_vector_get(vec, i);
  }
  return myprod;
}



gsl_vector* tklib_matrix_sum(gsl_matrix* A, int dimension){
  size_t ysize; 
  size_t xsize;
  
  
  if(dimension == 1){
    ysize = A->size1;
    xsize = A->size2;
  }
  else{
    ysize = A->size2;
    xsize = A->size1;
  }
  gsl_vector* sums = gsl_vector_calloc(ysize);

  size_t i;
  for(i = 0; i < ysize; i++){
    size_t j;    
    for(j = 0; j < xsize; j++)
      if(dimension == 1)
	gsl_vector_set(sums, i, gsl_vector_get(sums, i)+gsl_matrix_get(A, i, j));
      else
	gsl_vector_set(sums, i, gsl_vector_get(sums, i)+gsl_matrix_get(A, j, i));
  }

  return sums;
}

gsl_vector* tklib_matrix_prod(gsl_matrix* A, int dimension){
  size_t ysize; 
  size_t xsize;
  
  
  if(dimension == 1){
    ysize = A->size1;
    xsize = A->size2;
  }
  else{
    ysize = A->size2;
    xsize = A->size1;
  }
  gsl_vector* prods = gsl_vector_calloc(ysize);
  gsl_vector_set_all(prods, 1.0);

  size_t i;
  for(i = 0; i < ysize; i++){
    size_t j;    
    for(j = 0; j < xsize; j++)
      if(dimension == 1)
	gsl_vector_set(prods, i, gsl_vector_get(prods, i)*gsl_matrix_get(A, i, j));
      else
	gsl_vector_set(prods, i, gsl_vector_get(prods, i)*gsl_matrix_get(A, j, i));
  }

  return prods;
}



gsl_vector* tklib_matrix_min(gsl_matrix* A, int dimension){
  size_t ysize; 
  size_t xsize;
  
  
  if(dimension == 1){
    ysize = A->size1;
    xsize = A->size2;
  }
  else{
    ysize = A->size2;
    xsize = A->size1;
  }
  gsl_vector* mins = gsl_vector_calloc(ysize);
  gsl_vector_add_constant(mins, 1000000000);

  size_t i;
  for(i = 0; i < ysize; i++){
    size_t j;    
    for(j = 0; j < xsize; j++)
      if(dimension == 1)
	gsl_vector_set(mins, i, fmin(gsl_vector_get(mins, i), gsl_matrix_get(A, i, j)));
      else{
	//fprintf(stderr, "min:%f", fmin(gsl_vector_get(mins, i), gsl_matrix_get(A, j, i)));
	gsl_vector_set(mins, i, fmin(gsl_vector_get(mins, i), gsl_matrix_get(A, j, i)));
      }
  }

  return mins;
}

gsl_vector* tklib_matrix_argmin(gsl_matrix* A, int dimension){
  //dimension == 1 is the height of the matrix
  //dimension == 0 is the width of the matrix
  int length=-1;
  if(dimension ==1){
    length = A->size1;
  }
  else if(dimension ==0){
    length = A->size2;
  }

  gsl_vector* argmins = gsl_vector_calloc(length);
  
  size_t i;
  for(i=0;(int)i<length;i++){
    if(dimension == 1){
      gsl_vector_view v = gsl_matrix_row(A, i);
      size_t myindex = gsl_vector_min_index(&v.vector);
      gsl_vector_set(argmins, i, myindex);
    }
    else if(dimension == 0){
      gsl_vector_view v = gsl_matrix_column(A, i);
      size_t myindex = gsl_vector_min_index(&v.vector);
      gsl_vector_set(argmins, i, myindex);
    }
  }

  return argmins;
}

void tklib_permutation_printf(gsl_permutation* theperm){
  size_t i;
  for(i=0;i<theperm->size;i++){
    printf("%lu\t", gsl_permutation_get(theperm, i));
  }
  printf("\n");
}

void tklib_vector_printf(gsl_vector* thevec){
  size_t i;
  for(i=0;i<thevec->size;i++){
    printf("%.66f\t", gsl_vector_get(thevec, i));
  }
  printf("\n");

}

/*gsl_vector* tklib_marray_get_row(marray* myarray, int dim, size_t* index){
  gsl_vector* ret_array=gsl_vector_calloc(myarray->dimension[dim]);
  size_t* ch_index = index + dim;
  
  for(size_t i=0; i<myarray->dimension[dim]; i++){
    *ch_index=i;
    
    double val = marray_get(myarray, index);
    gsl_vector_set(ret_array, i, val);
  }
  
  return ret_array;
}


gsl_matrix* tklib_marray_2matrix(marray* t){
  gsl_matrix * m;
  
  if (t->rank != 2)
    GSL_ERROR_NULL("marray of rank != 2", GSL_EINVAL);
  
  
  m = (gsl_matrix*)malloc(sizeof(gsl_matrix));
  if (m == 0)
    GSL_ERROR_VAL ("failed to allocate space for matrix struct",
                   GSL_ENOMEM, 0);
  
  m->data = t->data;
  m->size1 = t->dimension[0];
  m->size2 = t->dimension[1];
  m->tda = t->dimension[1];
  m->block = NULL;  // note that this is no problem because owner=0 
  m->owner = 0;
  
  return m;
}*/

void tklib_matrix_printf(gsl_matrix* themat){

  printf("size:%lu,%lu\n", themat->size1, themat->size2);
  size_t i;
  for(i=0;i<themat->size1;i++){
    size_t j;
    for(j=0;j<themat->size2;j++){
      printf("%.66f\t", gsl_matrix_get(themat, i, j));
    }
    printf("\n");
  }
  
}
void tklib_matrix_float_printf(gsl_matrix_float* themat){
  size_t i;
  for(i=0;i<themat->size1;i++){
    size_t j;
    for(j=0;j<themat->size2;j++){
      printf("%f\t", gsl_matrix_float_get(themat, i, j));
    }
    printf("\n");
  }
}
void tklib_const_matrix_printf(const gsl_matrix* themat){
  size_t i;
  for(i=0;i<themat->size1;i++){
    size_t j;
    for(j=0;j<themat->size2;j++){
      printf("%f\t", gsl_matrix_get(themat, i, j));
    }
    printf("\n");
  }
}

void tklib_matrix_sqrt(gsl_matrix* X){
  size_t i;
  for(i = 0; i < X->size1; i++){
    size_t j;    
    for(j = 0; j < X->size2; j++)
      gsl_matrix_set(X, i, j, sqrt(gsl_matrix_get(X, i,j)));
  }
}

void tklib_vector_sqrt(gsl_vector* vec){
  size_t i;
  for(i = 0; i < vec->size; i++)
    gsl_vector_set(vec, i, sqrt(gsl_vector_get(vec, i)));
}


gsl_matrix* tklib_matrix_mul_vec(gsl_matrix* pts, gsl_vector *vec){
  gsl_matrix* retpts = gsl_matrix_calloc(pts->size1, pts->size2);
  gsl_matrix_memcpy(retpts, pts);

  tklib_matrix_mul_vec_inplace(retpts, vec);

  return retpts;
}

void tklib_matrix_mul_vec_inplace(gsl_matrix* pts, gsl_vector *vec){
  size_t j;    
  for(j = 0; j < pts->size2; j++){
    gsl_vector_view v = gsl_matrix_column(pts, j);
    gsl_vector_mul(&v.vector, vec);
  }
}



//This assumes that its a column vector and will be added columnwise
//pts is modified in place
//alpha is a constant to multiply by the first element
//beta is a constant to multiply by the second element
void tklib_matrix_add_vec(gsl_matrix* pts, gsl_vector *vec, double alpha, double beta){
  size_t i;
  for(i = 0; i < pts->size1; i++){
    size_t j;    
    for(j = 0; j < pts->size2; j++)
      gsl_matrix_set(pts, i, j, alpha*gsl_matrix_get(pts, i, j) + beta*gsl_vector_get(vec, i));
  }
}



//This assumes that its a column vector and will be added columnwise
//pts is modified in place
//alpha is a constant to multiply by the first element
//beta is a constant to multiply by the second element
void tklib_matrix_add_vec_M(gsl_matrix* pts, gsl_matrix *vec, double alpha, double beta){
  size_t i;
  for(i = 0; i < pts->size1; i++){
    size_t j;    
    for(j = 0; j < pts->size2; j++)
      gsl_matrix_set(pts, i, j, alpha*gsl_matrix_get(pts, i, j) + beta*gsl_matrix_get(vec, i, 0));
  }
}




void tklib_vector_intersect(gsl_vector * mask1, gsl_vector * mask2, gsl_vector * dest)
{
  assert(mask1->size == mask2 -> size);
  assert(mask1->size == dest -> size);
  size_t i;
  for (i = 0; i < mask1->size; i++) {
    if (gsl_vector_get(mask1, i) && 
	gsl_vector_get(mask2, i)) {
      gsl_vector_set(dest, i, 1);
    } else {
      gsl_vector_set(dest, i, 0);
    }
  }
}



void tklib_vector_union(gsl_vector * mask1, gsl_vector * mask2, gsl_vector * dest)
{
  assert(mask1->size == mask2 -> size);
  assert(mask1->size == dest -> size);
  size_t i;
  for (i = 0; i < mask1->size; i++) {
    if (gsl_vector_get(mask1, i) || 
	gsl_vector_get(mask2, i)) {
      gsl_vector_set(dest, i, 1);
    } else {
      gsl_vector_set(dest, i, 0);
    }
  }
}


void tklib_apply_mask_lp(gsl_vector * mask, gsl_vector * log_probs, gsl_vector * dest) 
{
  assert(mask->size == log_probs -> size);
  for (size_t i = 0; i < log_probs->size; i++) {
    double mask_val = gsl_vector_get(mask, i);
    if ( mask_val == 0) {
      gsl_vector_set(dest, i, GSL_NEGINF);
    } else { 
      assert(mask_val == 1);
      gsl_vector_set(dest, i, gsl_vector_get(log_probs, i));
    }
  }
}

gsl_matrix * tklib_ones(int d1, int d2)
{
  gsl_matrix * result = gsl_matrix_alloc(d1, d2);
  gsl_matrix_set_all(result, 1);
  return result;
}

gsl_matrix * tklib_transpose(gsl_matrix * m)
{
  gsl_matrix * result = gsl_matrix_alloc(m->size2, m->size1);
  gsl_matrix_transpose_memcpy(result, m);
  return result;
}

gsl_matrix * tklib_matrix_dot(gsl_matrix * m1, gsl_matrix *m2)
{

  gsl_matrix * result = gsl_matrix_alloc(m1->size1, m2->size2);
  gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                  1.0, m1, m2,
                  0.0, result);
  return result;
}






double tklib_vector_mean(gsl_vector * vector) 
{
  double total = 0.0;
  for (size_t i = 0; i < vector->size; i++) {
    total += gsl_vector_get(vector, i);
  }
  return total / vector->size;
}

double tklib_vector_variance(gsl_vector * vector) 
{
  double mean = tklib_vector_mean(vector);
  double totalSquared = 0.0;
  for (size_t i = 0; i < vector->size; i++) {
    double val = gsl_vector_get(vector, i);
    totalSquared += val * val;
  }
  return totalSquared / vector->size -  mean * mean;
}

double tklib_vector_stddev(gsl_vector * vector) 
{
  double variance = tklib_vector_variance(vector);
  return pow(variance, 0.5);
}

/**
 * Returns the argmin of the vector, or -1 if the vector is empty.
 */
int tklib_vector_argmin(gsl_vector * vector) 
{
  double min_value = GSL_POSINF;
  int min_idx = -1;
  for (size_t i = 0; i < vector->size; i++) {
    double value = gsl_vector_get(vector, i);
    if (value < min_value) {
      min_value = value;
      min_idx = i;
    }
  } 
 return min_idx;
}



/**
 * Returns the argmax of the vector, or -1 if the vector is empty.
 */
int tklib_vector_argmax(gsl_vector * vector) 
{
  double max_value = GSL_NEGINF;
  int max_idx = -1;
  for (size_t i = 0; i < vector->size; i++) {
    double value = gsl_vector_get(vector, i);
    if (value > max_value) {
      max_value = value;
      max_idx = i;
    }
  }
  return max_idx;
}


double tklib_vector_dot(gsl_vector * v1, gsl_vector * v2)
{
  assert(v1->size == v2->size);
  double result = 0.0;
  for (size_t i = 0; i < v1->size; i++) {
    result += gsl_vector_get(v1, i) * gsl_vector_get(v2, i);
  }
  return result;

}

double tklib_vector_equal(gsl_vector * v1, gsl_vector * v2) 
{
  if (v1->size != v2->size) {
    return false;
  }
  for (size_t i = 0; i < v1->size; i++) {
    if (gsl_vector_get(v1, i) != gsl_vector_get(v2, i)) {
      return false;
    }
  }
  return true;
}


gsl_vector * tklib_vector_linspace(double start, double stop, int num_units)
{
  int num_indices = num_units;
  double step_size = (stop - start) / (num_units - 1);
  gsl_vector * result = gsl_vector_alloc(num_indices);
  for (int i = 0; i < num_indices; i++) {
    double value = start + step_size * i;
    gsl_vector_set(result, i, value);
  }
  //printf("size: %d\n", num_indices);
  return result;
}
int tklib_vector_bisect(gsl_vector * v, double value)
{
  int lower = 0;
  int upper = v->size;
  
  unsigned int current_idx;
  while (true) {
    current_idx = lower + (upper - lower) / 2;
    double current_value = gsl_vector_get(v, current_idx);
    //printf("[%d, %d], %.3f\n", lower, upper, current_value);
    //math2d_vector_printf(v);
    if (value < current_value) {
      upper = current_idx;
    } else if (value > current_value) {
      lower = current_idx;
    } else {
      break;
    }

    if (upper == lower || upper - lower == 1) {
      current_idx = upper;
      break;
    }
  }
  if (current_idx < v->size && gsl_vector_get(v, current_idx) == value) {
    current_idx ++;
  }
  if (value < gsl_vector_get(v, 0)) {
    current_idx = 0;
  }

  return current_idx;
}
