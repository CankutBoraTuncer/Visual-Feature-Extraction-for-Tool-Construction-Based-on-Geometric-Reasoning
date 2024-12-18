import itertools
import numpy as np

class Score():
    def __init__(self, reference_params, candidate_params, verbose = 0):
        self.reference_params = reference_params
        self.candidate_params = candidate_params
        self.verbose = verbose

    def calculate_action_score(self):
        score = 0
        for ref, cand in zip(self.reference_params, self.candidate_params):
            shape_diff = sum(abs(cand[3:5] - ref[3:5]))
            position_diff = sum(abs(cand[-3:] - ref[-3:]))
            
            score += shape_diff + position_diff
        return score


    def find_best_fit(self):
        num_parts = len(self.reference_parts)
        best_score = float('inf')
        best_permutation = None
        best_mapping = []

        # Generate all permutations
        permutations = itertools.permutations(self.candidate_params, num_parts)

        for perm in permutations:
            candidate_params = np.array(perm)
            score = self.calculate_action_score()
            if score < best_score:
                best_score = score
                best_permutation = candidate_params
                best_mapping = [(f"Reference Part {i+1}", f"Candidate Object {self.candidate_params.tolist().index(p.tolist()) + 1}") 
                                for i, p in enumerate(perm)]

        if self.verbose > 0:
            print("Best Permutation (Matched Candidate Parameters):")
            print(best_permutation)
            print(f"\nBest Score: {best_score}")
            print("\nDetailed Matches:")
            for match in best_mapping:
                print(f"{match[0]} matches with {match[1]}")

        return best_permutation, best_score, best_mapping


