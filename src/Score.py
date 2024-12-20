import itertools
import numpy as np

class Score():
    def __init__(self, reference_params, candidate_params, verbose=0):
        self.reference_params = reference_params  
        self.candidate_params = candidate_params  
        self.verbose = verbose
        self.weight_shape = 1
        self.weight_size = 1
        self.weight_proportion = 1

    # ---------------------------------------------------------------------------------------# 
    # ---------------------------------------------------------------------------------------#
    # ---------------------------------------------------------------------------------------#

    def algorithm_1(self):
        error_list = []
        
        candidate_names = list(self.candidate_params.keys())
        candidate_values = list(self.candidate_params.values())
        
        candidate_perm = list(itertools.permutations(candidate_values, 2))
        
        for perm in candidate_perm:
            total_error = 0
            shape_error = 0
            size_error = 0
            ratio_error = 0
            
            for j, cand in enumerate(perm):
                ref_name = list(self.reference_params.keys())[j]  
                ref = self.reference_params[ref_name]  
                
                shape_error += np.linalg.norm(cand[4:6] - ref[4:6])
                size_error += np.linalg.norm(cand[0:3] - ref[0:3])
                
                for k in range(len(perm)):
                    if k != j:  
                        ratio_error += abs((cand[0] / cand[1]) - (perm[k][0] / perm[k][1]))
            
            total_error = (
                self.weight_shape * shape_error + 
                self.weight_size * size_error + 
                self.weight_proportion * ratio_error
            )
            error_list.append(total_error)
        
        sorted_indices = np.argsort(error_list)
        
        sorted_T = [tuple(candidate_names[idx] for idx in perm) for perm in list(itertools.permutations(range(len(candidate_names)), 2))]
        sorted_T = [sorted_T[i] for i in sorted_indices]  
        
        if self.verbose>0:
            print(f"{list(self.reference_params.keys())[0]} object matches with {sorted_T[0][0]} object")
            print(f"{list(self.reference_params.keys())[1]} object matches with {sorted_T[0][1]} object")
        
        return [list(self.reference_params.keys())[0], sorted_T[0][0]], [list(self.reference_params.keys())[1], sorted_T[0][1]]
