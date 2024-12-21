import itertools
import numpy as np

class Score():
    def __init__(self, reference_params, candidate_params, verbose=0):
        self.reference_params = reference_params  
        self.candidate_params = candidate_params  
        self.verbose = verbose
        self.weight_shape = 1
        self.weight_size = 1
        self.weight_proportion = 0.5

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
                
                shape_error += np.linalg.norm([ref[4] - cand[4], ref[5] - cand[5]], ord=2)
                size_error += np.linalg.norm([ref[0] - cand[0], 
                             ref[1] - cand[1], 
                             ref[2] - cand[2]], ord=2)
                
                for k in range(len(perm)):
                    if k != j:  

                        ratio1_ref = ref[0] / ref[1]
                        ratio2_ref = ref[0] / ref[2]
                        ratio3_ref = ref[1] / ref[2]

                        ratio1_cand = cand[0] / cand[1]
                        ratio2_cand = cand[0] / cand[2]
                        ratio3_cand = cand[1] / cand[2]
                        
                        ratio_error += ((ratio1_ref - ratio1_cand) ** 2 + 
                                                (ratio2_ref - ratio2_cand) ** 2 + 
                                                (ratio3_ref - ratio3_cand) ** 2) ** 0.5    
            total_error = (
                self.weight_shape * shape_error + 
                self.weight_size * size_error + 
                self.weight_proportion * ratio_error
            )

            error_list.append((perm, total_error, shape_error, size_error, ratio_error))
        
        error_list.sort(key=lambda x: x[1])
        
        sorted_T = []
        for perm, score, shape_error, size_error, ratio_error in error_list:
            matches = [
                (list(self.reference_params.keys())[i], 
                candidate_names[next(idx for idx, cand in enumerate(candidate_values) if np.array_equal(cand, p))])
                for i, p in enumerate(perm)
            ]
            sorted_T.append(matches)
            if self.verbose > 0:
                print(f"Matches: {matches}, Total Error: {score:.4f}, Shape Error: {shape_error:.4f}, Size Error: {size_error:.4f}, Ratio Error: {ratio_error:.4f}")

        return sorted_T[0][0], sorted_T[0][1]
