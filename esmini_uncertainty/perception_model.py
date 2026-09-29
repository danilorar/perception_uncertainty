"""perception uncertainty model using heteroscedastic measurement-error model"""

import random # for seed 
from dataclasses import dataclass # data structure 

@dataclass(frozen=True) 
class DistanceErrorBin: 
    """distance error bin"""

    min_distance: float
    max_distance: float
    mean_error: float
    std_error: float
    
DISTANCE_ERROR_BINS = [
    
    # BIN 1: rel_distance = [0, 25], mean_error = 0.0, std_error = 0.1
    DistanceErrorBin(min_distance=0.0, max_distance=25.0, mean_error=0.0, std_error=0.1),
    
    # BIN 2: rel_distance = [25,50], mean_error = 0.0, std_error = 0.3
    DistanceErrorBin(min_distance=25.0, max_distance=50.0, mean_error=0.0, std_error=0.3),
    
    # BIN 3: rel_distance = [50,75], mean_error = 0.0, std_error = 0.8
    DistanceErrorBin(min_distance=50.0, max_distance=75.0, mean_error=0.0, std_error=0.8),
    
    # BIN 4: rel_distance = [75,100], mean_error = 0.0, std_error = 1.5
    DistanceErrorBin(min_distance=75.0, max_distance=100.0, mean_error=0.0, std_error=1.5),
]
    
    
class PerceptionModel: 
    """perception uncertainty model"""

    def __init__(self, seed: int = 42): 
        self.random_generator = random.Random(seed)  
        
    # Method to select error bin based on true distance
    def _select_error_bin(self, true_distance): 
        for bin in DISTANCE_ERROR_BINS:
            
            # If the true distance falls within the range of the current bin, return that bin
            if bin.min_distance <= true_distance < bin.max_distance:
                return bin 
            
        return DISTANCE_ERROR_BINS[-1]  # Return the last bin if no match is found
            
    
    def measure_distance(self, true_distance): 
        error_bin = self._select_error_bin(true_distance)
        
        # Create a Gaussian distribution based on the mean and standard deviation of the selected error bin
        distance_error = self.random_generator.gauss(
            mu=error_bin.mean_error,  # mean error for the selected bin
            sigma=error_bin.std_error # standard deviation for the selected bin 
        )
        
        # Ensure perceived distance is non-negative
        perceived_distance = max(0.0, true_distance + distance_error)  
        
        return {
            "true_distance": true_distance,
            "perceived_distance": perceived_distance,
            "distance_error": perceived_distance - true_distance,
            "error_bin": error_bin
        }