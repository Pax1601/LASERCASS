#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream.h>                     // math routines
#include "ANN.h"                    // ANN declarations

extern "C" {
static ANNkd_tree*   the_tree;              // search structure
unsigned int n_pts;
unsigned int dims;
ANNpointArray data_pts;               // data points

int *nearn; 
double *distn;
double *str_data=NULL;
double *aero_data=NULL;


int add_pts(double* data, unsigned int n_row, unsigned int dim)
{
        
        data_pts = annAllocPts(n_row, dim);         // allocate data points
        n_pts = n_row;
        dims = dim;
        for (unsigned int i=0; i < n_row; i++) {
                for (unsigned int k=0; k < dim; k++) {
                        data_pts[i][k] = data[k*n_row+i];
                }
        }
        return 0;
}

int create_tree(void)
{
        the_tree = new ANNkd_tree(              // build search structure
                        data_pts,                       // the data points
                        n_pts,                  // number of points
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
        the_tree->annkSearch(query, k, idx, dist,eps);  // search
        return 0;
}

/* priority search */
int query_treeiPri(int pos, double* query, unsigned int k, int* idx, double* dist, double eps = 0.0)
{
        the_tree->annkSearch(query, k, idx, dist,eps);  // search
        return 0;
}

//int dump_tree_(int pos) {
//        the_tree->Dump(ANNtrue, std::cout);
//        return 0;
//}
/*
 d dimensione fisica dello spazio 
 kn il numero di nodi vicini da ricercare 
 datain struttura contenente le coordinate dei nodi di base 
 sz numero dei nodi di base
 queryp struttura contenente le coordinate dei nodi ricerca 
 qn numero nodi di ricerca 
 ritorna:
 nearn label nodi piu' vicini
 distn distanze nodi piu' vicini
*/
/*
void knearn(int* d, int* kn, double* datain, int* sz,  double* queryp, int* qN, 
		int* nearn, double* distn, int* error)
*/
void knearn(int dim, int k, double* datain, int sz,  double* queryp, int qN, int error)

{
    
    double* query;
    int* idx;
    double* dist;
    int i,j;
    double eps;
     
    eps = 0.0; 

  
    idx = (int *) malloc(sizeof(int)*k);
    dist = (double *) malloc(sizeof(double)*k);
    query = (double *) malloc(sizeof(double)*dim);


    error = add_pts(datain,sz,dim); 
    if (error == -1) 
	{
        printf("\nKNEARN error.\n");
        free(idx);
        free(dist);	
        free(query);
		return;
    }
    error = create_tree();
    if (error == -1) {
        printf("\nKNEARN error.\n");
        free(idx);
        free(dist);	
        free(query);
	return;
    }
    
    for (i = 0; i < qN; i++) 
	{
		for (j = 0; j < dim; j++) 
		{
			query[j] = queryp[j*qN+i];
		}
		error = query_tree(query,k,idx,dist,eps);
		if (error == -1) 
		{
			printf("KNEARN error.\n");
			free(idx);
        	free(dist);	
        	free(query);
			return;
		}
		for (j=0; j < k; j++) 
		{
			nearn[j*qN+i] = idx[j]+1;
			distn[j*qN+i] = dist[j];
		}
	}
	error = delete_tree();	
	free(idx);
	free(dist);	
	free(query);
}
}

void main(void)
{
double node_cord;
int strN, aerN, dim, points, i, j;
FILE *fp;
char file_int[50];
int error=0;
printf("\n********************************************************************************");
printf("\nSimSAC Project");
printf("\nDepartment of Aerospace Engineering - Politecnico di Milano (DIAPM)");
printf("\nWarning: This code is released only to be used by SimSAC partners.");
printf("\nAny usage without an explicit authorization may be persecuted.");
printf("\n");
printf("\n********************************************************************************");


if ((fp = fopen("param.int","r")) == NULL)
{
	printf("\nWarning: cannot find the file containing interface parameters. Aborting.");
	return;
}

fscanf(fp,"%s %d",&file_int,&dim);
printf("\n\nSize of space: %d",dim);
fscanf(fp,"%s %d",&file_int,&points);
printf("\nNumber of k nearest points required: %d",points);
fclose(fp);

if ((fp = fopen("query_input.int","r")) == NULL)
{
	printf("\nWarning: cannot find the file containing the query nodes. Aborting.");
	return ;
}
printf("\nReading query nodes...");
fscanf(fp,"%s %d",&file_int,&aerN);
aero_data = (double *)calloc(aerN*dim,sizeof(double));

for (i=0;i<aerN;++i)
for (j=0;j<dim;++j)
{
	fscanf(fp,"%lf",&node_cord);
	aero_data[i+j*aerN] = node_cord;
}

fclose(fp);
printf("done. Found %d nodes.",aerN);
	
if ((fp = fopen("basic_input.int","r")) == NULL)
{
	printf("\nWarning: cannot find the file containing the basic nodes. Aborting.");
	return ;
}
printf("\nReading basic nodes...");
fscanf(fp,"%s %d",&file_int,&strN);
str_data = (double *)calloc(strN*dim,sizeof(double));

for (i=0;i<strN;++i)
for (j=0;j<dim;++j)
{
	fscanf(fp,"%lf",&node_cord);
	str_data[i+j*strN] = node_cord;
}
fclose(fp);
printf("done. Found %d nodes.\n",strN);

nearn=(int *)calloc(aerN*points,sizeof(int));
distn=(double *)calloc(aerN*points,sizeof(double));

knearn(dim, points, str_data, strN, aero_data, aerN, error);

fp=fopen("nnpos.int","w");
for (i=0;i<aerN;i++)
{
	for (j=0;j<points;j++)
{
	fprintf(fp,"%d ",nearn[j*aerN+i]);
}
fprintf(fp,"\n");
}
fclose(fp);

fp=fopen("dist.int","w");
for (i=0;i<aerN;i++)
{
	for (j=0;j<points;j++)
{
	fprintf(fp,"%10.9lf ",distn[j*aerN+i]);
}
fprintf(fp,"\n");
}
fclose(fp);

printf("\n\n");
free(aero_data);
free(str_data);
free(nearn);
free(distn);
}
