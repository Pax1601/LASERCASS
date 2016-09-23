/*
#***********************************************************************************************************************
#  SimSAC Project
#
#  SMARTCAD
#  Simplified Models for Aeroelasticity in Conceptual Aircraft Design  
#
#                      Sergio Ricci         <ricci@aero.polimi.it>
#                      Luca Cavagna         <cavagna@aero.polimi.it>
#                      Alessandro Degaspari <degaspari@aero.polimi.it>
#                      Luca Riccobene       <riccobene@aero.polimi.it>
#
#  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
#  Warning: This code is released only to be used by SimSAC partners.
#  Any usage without an explicit authorization may be persecuted.
#
#***********************************************************************************************************************
#
#	KNEARN.C  .MEX File to compute the k nearest neighbours for a set of query points given the set of search points.
#
#   Usage:
#   	[nnp] = knearn(pos, indata, query, k, dim);
#
#	Input:
#
#   indata	matrix containing data points position [pointsN x dim]
#   query	query points [qpintsN x dim]
#	dim dimensions of points space
#   k	number of nearest neighbours
#	
# This is a MEX-file for MATLAB.  
# For the ANN package used see the copyright note in the README file of the ANN Package
# Copyright (c) 1997-1998 University of Maryland and Sunil Arya and David Mount
# All Rights Reserved. 
# 
#**********************************************************************************************************************/
/* $Revision: 0.1 $ */        

#include <math.h>
#include "mex.h"
#include "matrix.h"
#include <stdio.h>

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] )

{
    double *datain;
    double *queryp;
    double *out_pos;	
    double *out_dist;	
    unsigned int dim, k;
    unsigned int dataN, queryN, n;
    double* query;
    int* idx;
    double* dist;
    int i,j;
    double* d;
    int error;
    double eps;
     
    eps = 0.0; 
    /* Check for proper number of arguments */

    if ((nrhs < 4) || (nrhs > 5)) {
        mexPrintf("KNEARN requires four or five inputs\n");
	return;
    } else if (nlhs > 2) {
        mexPrintf("KNEARN too many outputs\n");
	return;
    }

    /* Check the type and dimensions of data  */
    
    d =  mxGetPr(prhs[2]);
    dim = (unsigned int) d[0]; 
    d =  mxGetPr(prhs[3]);
    k = (unsigned int) d[0]; 
    
  
    if (dim <= 0) {
        mexPrintf("KNEARN requires that DIM > 0\n");
	return;
    }
    if (k <= 0) {
        mexPrintf("KNEARN requires that K > 0\n");
	return;
    }
    if (nrhs > 4) {
    	eps =  *mxGetPr(prhs[4]);
    }

    dataN = mxGetM(prhs[0]);
    queryN = mxGetM(prhs[1]);
    plhs[0] = mxCreateDoubleMatrix(queryN, k, mxREAL);
    if (nlhs > 1) plhs[1] = mxCreateDoubleMatrix(queryN, k, mxREAL);
	

    n = mxGetN(prhs[0]);
    if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || (n != dim) || (dataN == 0)) {
	mexPrintf("KNEARN requires that INDATA be a N x dim real matrix.\n");
	return;
    }

    n = mxGetN(prhs[1]);
    if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || (n != dim) || (queryN == 0)) {
        mexPrintf("KNEARN requires that QUERY be a N x dim real matrix.\n");
	return;
    }

    datain  = mxGetPr(prhs[0]);	
    queryp = mxGetPr(prhs[1]);	
    idx = malloc(sizeof(int)*k);
    dist = malloc(sizeof(double)*k);
    query = malloc(sizeof(double)*dim);

    /* Create a matrix for the return argument */
    /* Assign pointers to the various parameters */

    out_pos = mxGetPr(plhs[0]);
    
    if (nlhs > 1) {
	out_dist = mxGetPr(plhs[1]);
    } else {
    	out_pos = malloc(sizeof(double)*queryN*k);
    }

    error = add_pts(datain, dataN,dim); 
    if (error == -1) {
        mexPrintf("KNEARN error.\n");
        free(idx);
        free(dist);	
        free(query);
	return;
    }
    error = create_tree();
    if (error == -1) {
        mexPrintf("KNEARN error.\n");
        free(idx);
        free(dist);	
        free(query);
	return;
    }
    
    for (i = 0; i < queryN; i++) {
     	for (j = 0; j < dim; j++) {
		query[j] = queryp[j*queryN+i]; 
	}
	
	error = query_tree(query, k, idx, dist,eps);
    	
    	
    	if (error == -1) {
        	mexPrintf("KNEARN error.\n");
        	free(idx);
        	free(dist);	
        	free(query);
		return;
    	}
	for (j=0; j < k; j++) {
		out_pos[j*queryN+i] = (double)(idx[j]+1);
		out_dist[j*queryN+i] = dist[j];
	}
     }
     error = delete_tree();	
     free(idx);
     free(dist);	
     free(query);

}
