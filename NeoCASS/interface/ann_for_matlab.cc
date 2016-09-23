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
*/

#include <iostream>			// math routines
#include "./ann_1.1.2/include/ANN/ANN.h"			// ANN declarations

extern "C" {
static ANNkd_tree*   the_tree;              // search structure
unsigned int n_pts;
unsigned int dims;
ANNpointArray data_pts;               // data points



int add_pts(double* data, unsigned int n_row, unsigned int dim)
{
	// controllo che pos sia vuoto 
        data_pts = annAllocPts(n_row, dim);         // allocate data points
	n_pts = n_row;
	dims = dim;
        /* riempie data_pts */
        for (int i=0; i < n_row; i++) {
		for (int k=0; k < dim; k++) {
			data_pts[i][k] = data[k*n_row+i];
		}
	}
	return 0;
}

int create_tree(void) 
{
    	the_tree = new ANNkd_tree(		// build search structure
		    	data_pts,			// the data points
		    	n_pts,			// number of points
		    	dims);
	return 0;
}

int delete_tree(void)
{
       	delete the_tree;
	delete data_pts;
	return 0;
}
	 	
/* normal search */		
int query_tree(double* query, unsigned int k, int* idx, double* dist, double eps = 0.0)
{
	the_tree->annkSearch(query, k, idx, dist,eps);	// search
	return 0;
}

/* priority search */		
int query_treeiPri(int pos, double* query, unsigned int k, int* idx, double* dist, double eps = 0.0)
{
	the_tree->annkSearch(query, k, idx, dist,eps);	// search
	return 0;
}

int dump_tree(int pos) {
	the_tree->Dump(ANNtrue, std::cout);
	return 0;
}

}	
