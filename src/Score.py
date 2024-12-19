import itertools
import numpy as np

class Score():
    def __init__(self, reference_params, candidate_params, verbose=0):
        self.reference_params = reference_params
        self.candidate_params = candidate_params
        self.verbose = verbose

    def calculate_action_score(self, ref_arrays, cand_arrays):
        score = 0
        for ref, cand in zip(ref_arrays, cand_arrays):
            shape_diff = sum(abs(cand[3:5] - ref[3:5]))
            position_diff = sum(abs(cand[-3:] - ref[-3:]))
            score += shape_diff + position_diff
        return score

    def find_best_fit(self):
        ref_keys = list(self.reference_params.keys())
        cand_keys = list(self.candidate_params.keys())
        
        num_parts = len(ref_keys)
        best_score = float('inf')
        best_permutation = None
        best_mapping = []

        for perm in itertools.permutations(cand_keys, num_parts):
            ref_arrays = [self.reference_params[k] for k in ref_keys]
            cand_arrays = [self.candidate_params[k] for k in perm]

            score = self.calculate_action_score(ref_arrays, cand_arrays)
            
            if score < best_score:
                best_score = score
                best_permutation = perm
                best_mapping = [[ref_key, cand_key] for ref_key, cand_key in zip(ref_keys, perm)]

        if self.verbose > 0:
            print("Best Permutation (Matched Candidate Parameters):")
            for ref_obj, cand_obj in best_mapping:
                print(f"{ref_obj} matches with {cand_obj}")
            print(f"\nBest Score: {best_score}")

        return best_mapping, best_permutation, best_score
