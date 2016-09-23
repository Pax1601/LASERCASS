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
# Requires GNU g++ and Matlab mex compilers
#
# Compile ANN library (see ANN documentation) and set ANNLIB variable pointing to ANN directory.
#
# Type "make" to build a mex file for Matlab.
#
# Type "make clean" to remove the built mex file.
# 
# Modify ANNLIB variable to correctly point to ANN library directory 
#
ANNLIB= ./ann_1.1.2
all: knearn.mexglx

knearn.mexglx: knearn.c ann_for_matlab.o
	mex -L $(ANNLIB)/lib knearn.c ann_for_matlab.o -lANN

ann_for_matlab.o: ann_for_matlab.cc
	g++ -I $(ANNLIB)/include/ANN -c ann_for_matlab.cc -fpic
	
clean: 
	rm -rf *.o *.mex*
