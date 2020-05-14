#internal_simpleMPC_solver_1 : A fast customized optimization solver.
#
#Copyright (C) 2013-2020 EMBOTECH AG [info@embotech.com]. All rights reserved.
#
#
#This software is intended for simulation and testing purposes only. 
#Use of this software for any commercial purpose is prohibited.
#
#This program is distributed in the hope that it will be useful.
#EMBOTECH makes NO WARRANTIES with respect to the use of the software 
#without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
#PARTICULAR PURPOSE. 
#
#EMBOTECH shall not have any liability for any damage arising from the use
#of the software.
#
#This Agreement shall exclusively be governed by and interpreted in 
#accordance with the laws of Switzerland, excluding its principles
#of conflict of laws. The Courts of Zurich-City shall have exclusive 
#jurisdiction in case of any dispute.
#
#def __init__():
'''
a Python wrapper for a fast solver generated by FORCES PRO v3.0.0

   OUTPUT = internal_simpleMPC_solver_1_py.internal_simpleMPC_solver_1_solve(PARAMS) solves a multistage problem
   subject to the parameters supplied in the following dictionary:
       PARAMS['p_1'] - column vector of length 3

   OUTPUT returns the values of the last iteration of the solver where
       OUTPUT['o_1'] - column vector of size 1
       OUTPUT['o_2'] - column vector of size 1

   [OUTPUT, EXITFLAG] = internal_simpleMPC_solver_1_py.internal_simpleMPC_solver_1_solve(PARAMS) returns additionally
   the integer EXITFLAG indicating the state of the solution with 
       1 - Optimal solution has been found (subject to desired accuracy)
       2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
       0 - Timeout - maximum number of iterations reached
      -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
      -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
     -10 - The convex solver could not proceed due to an internal error
    -100 - License error

   [OUTPUT, EXITFLAG, INFO] = internal_simpleMPC_solver_1_py.internal_simpleMPC_solver_1_solve(PARAMS) returns 
   additional information about the last iterate:
       INFO.it        - number of iterations that lead to this result
       INFO.it2opt    - number of convex solves
       INFO.res_eq    - max. equality constraint residual
       INFO.res_ineq  - max. inequality constraint residual
       INFO.pobj      - primal objective
       INFO.dobj      - dual objective
       INFO.dgap      - duality gap := pobj - dobj
       INFO.rdgap     - relative duality gap := |dgap / pobj|
       INFO.mu        - duality measure
       INFO.sigma     - centering parameter
       INFO.lsit_aff  - iterations of affine line search
       INFO.lsit_cc   - iterations of line search (combined direction)
       INFO.step_aff  - step size (affine direction)
       INFO.step_cc   - step size (centering direction)
       INFO.solvetime - Time needed for solve (wall clock time)

 See also COPYING

'''

import ctypes
import os
import numpy as np
import numpy.ctypeslib as npct
import sys

#_lib = ctypes.CDLL(os.path.join(os.getcwd(),'internal_simpleMPC_solver_1/lib/internal_simpleMPC_solver_1.so')) 
try:
	_lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'internal_simpleMPC_solver_1/lib/internal_simpleMPC_solver_1.so'))
	csolver = getattr(_lib,'internal_simpleMPC_solver_1_solve')
except:
	_lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'internal_simpleMPC_solver_1/lib/libinternal_simpleMPC_solver_1.so'))
	csolver = getattr(_lib,'internal_simpleMPC_solver_1_solve')

class internal_simpleMPC_solver_1_params_ctypes(ctypes.Structure):
#	@classmethod
#	def from_param(self):
#		return self
	_fields_ = [('p_1', ctypes.c_double * 3),
]

internal_simpleMPC_solver_1_params = {'p_1' : np.array([]),
}
params = {'p_1' : np.array([]),
}
internal_simpleMPC_solver_1_params_types = {'p_1' : np.float64,
}

class internal_simpleMPC_solver_1_outputs_ctypes(ctypes.Structure):
#	@classmethod
#	def from_param(self):
#		return self
	_fields_ = [('o_1', ctypes.c_double * 1),
('o_2', ctypes.c_double * 1),
]

internal_simpleMPC_solver_1_outputs = {'o_1' : np.array([]),
'o_2' : np.array([]),
}


class internal_simpleMPC_solver_1_info(ctypes.Structure):
#	@classmethod
#	def from_param(self):
#		return self
	_fields_ = [('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('pobj',ctypes.c_double),
('dobj',ctypes.c_double),
('dgap',ctypes.c_double),
('rdgap',ctypes.c_double),
('mu',ctypes.c_double),
('mu_aff',ctypes.c_double),
('sigma',ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff',ctypes.c_double),
('step_cc',ctypes.c_double),
('solvetime',ctypes.c_double)
]

class FILE(ctypes.Structure):
        pass
if sys.version_info.major == 2:
	PyFile_AsFile = ctypes.pythonapi.PyFile_AsFile # problem here with python 3 http://stackoverflow.com/questions/16130268/python-3-replacement-for-pyfile-asfile
	PyFile_AsFile.argtypes = [ctypes.py_object]
	PyFile_AsFile.restype = ctypes.POINTER(FILE)

# determine data types for solver function prototype 
csolver.argtypes = ( ctypes.POINTER(internal_simpleMPC_solver_1_params_ctypes), ctypes.POINTER(internal_simpleMPC_solver_1_outputs_ctypes), ctypes.POINTER(internal_simpleMPC_solver_1_info), ctypes.POINTER(FILE))
csolver.restype = ctypes.c_int

def internal_simpleMPC_solver_1_solve(params_arg):
	'''
a Python wrapper for a fast solver generated by FORCES PRO v3.0.0

   OUTPUT = internal_simpleMPC_solver_1_py.internal_simpleMPC_solver_1_solve(PARAMS) solves a multistage problem
   subject to the parameters supplied in the following dictionary:
       PARAMS['p_1'] - column vector of length 3

   OUTPUT returns the values of the last iteration of the solver where
       OUTPUT['o_1'] - column vector of size 1
       OUTPUT['o_2'] - column vector of size 1

   [OUTPUT, EXITFLAG] = internal_simpleMPC_solver_1_py.internal_simpleMPC_solver_1_solve(PARAMS) returns additionally
   the integer EXITFLAG indicating the state of the solution with 
       1 - Optimal solution has been found (subject to desired accuracy)
       2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
       0 - Timeout - maximum number of iterations reached
      -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
      -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
      -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
     -10 - The convex solver could not proceed due to an internal error
    -100 - License error

   [OUTPUT, EXITFLAG, INFO] = internal_simpleMPC_solver_1_py.internal_simpleMPC_solver_1_solve(PARAMS) returns 
   additional information about the last iterate:
       INFO.it        - number of iterations that lead to this result
       INFO.it2opt    - number of convex solves
       INFO.res_eq    - max. equality constraint residual
       INFO.res_ineq  - max. inequality constraint residual
       INFO.pobj      - primal objective
       INFO.dobj      - dual objective
       INFO.dgap      - duality gap := pobj - dobj
       INFO.rdgap     - relative duality gap := |dgap / pobj|
       INFO.mu        - duality measure
       INFO.sigma     - centering parameter
       INFO.lsit_aff  - iterations of affine line search
       INFO.lsit_cc   - iterations of line search (combined direction)
       INFO.step_aff  - step size (affine direction)
       INFO.step_cc   - step size (centering direction)
       INFO.solvetime - Time needed for solve (wall clock time)

 See also COPYING

	'''
	global _lib

	# convert parameters
	params_py = internal_simpleMPC_solver_1_params_ctypes()
	for par in params_arg:
		try:
			#setattr(params_py, par, npct.as_ctypes(np.reshape(params_arg[par],np.size(params_arg[par]),order='A'))) 
			params_arg[par] = np.require(params_arg[par], dtype=internal_simpleMPC_solver_1_params_types[par], requirements='F')
			setattr(params_py, par, npct.as_ctypes(np.reshape(params_arg[par],np.size(params_arg[par]),order='F')))  
		except:
			raise ValueError('Parameter ' + par + ' does not have the appropriate dimensions or data type. Please use numpy arrays for parameters.')
    
	outputs_py = internal_simpleMPC_solver_1_outputs_ctypes()
	info_py = internal_simpleMPC_solver_1_info()
	if sys.version_info.major == 2:
		if sys.platform.startswith('win'):
			fp = None # if set to none, the solver prints to stdout by default - necessary because we have an access violation otherwise under windows
		else:
			#fp = open('stdout_temp.txt','w')
			fp = sys.stdout
		try:
			PyFile_AsFile.restype = ctypes.POINTER(FILE)
			exitflag = _lib.internal_simpleMPC_solver_1_solve( params_py, ctypes.byref(outputs_py), ctypes.byref(info_py), PyFile_AsFile(fp)  )
			#fp = open('stdout_temp.txt','r')
			#print (fp.read())
			#fp.close()
		except:
			#print 'Problem with solver'
			raise
	elif sys.version_info.major == 3:
		if sys.platform.startswith('win'):
			libc = ctypes.cdll.msvcrt
		elif sys.platform.startswith('darwin'):
			libc = ctypes.CDLL('libc.dylib')
		else:
			libc = ctypes.CDLL('libc.so.6')       # Open libc
		cfopen = getattr(libc,'fopen')        # Get its fopen
		cfopen.restype = ctypes.POINTER(FILE) # Yes, fopen gives a file pointer
		cfopen.argtypes = [ctypes.c_char_p, ctypes.c_char_p] # Yes, fopen gives a file pointer 
		fp = cfopen('stdout_temp.txt'.encode('utf-8'),'w'.encode('utf-8'))    # Use that fopen 

		try:
			if sys.platform.startswith('win'):
				exitflag = _lib.internal_simpleMPC_solver_1_solve( params_py, ctypes.byref(outputs_py), ctypes.byref(info_py), None )
			else:
				exitflag = _lib.internal_simpleMPC_solver_1_solve( params_py, ctypes.byref(outputs_py), ctypes.byref(info_py), fp )
			libc.fclose(fp)
			fptemp = open('stdout_temp.txt','r')
			print (fptemp.read())
			fptemp.close()			
		except:
			#print 'Problem with solver'
			raise

	# convert outputs
	for out in internal_simpleMPC_solver_1_outputs:
		internal_simpleMPC_solver_1_outputs[out] = npct.as_array(getattr(outputs_py,out))

	return internal_simpleMPC_solver_1_outputs,int(exitflag),info_py

solve = internal_simpleMPC_solver_1_solve


