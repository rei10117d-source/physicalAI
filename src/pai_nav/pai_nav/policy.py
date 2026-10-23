"""Pure Python k-nearest-neighbour classifier and deterministic expert teacher.
Inputs are normalised distances (0..1) for forward/left/right and signed goal bearing (-1..1).
Outputs are discrete actions: 0 forward, 1 turn left, 2 turn right.
"""
import json
import math

ACTIONS = ('forward','left','right')

def teacher(front,left,right,bearing):
    # A synthetic expert; the learned policy later imitates it on unseen observations.
    if front < 0.30:
        return 1 if left >= right else 2
    if front < 0.52 and abs(bearing) < 0.2:
        return 1 if left >= right else 2
    if bearing > 0.20:
        return 1
    if bearing < -0.20:
        return 2
    return 0

class KNNPolicy:
    def __init__(self, samples, k=5):
        self.samples = samples
        self.k = k

    @classmethod
    def from_file(cls,path):
        with open(path,encoding='utf-8') as file:
            obj=json.load(file)
        return cls(obj['samples'],obj['k'])

    def predict(self, features):
        # More weighting to forward clearance, moderate weighting to goal heading.
        weights=(2.0, 0.8, 0.8, 1.2)
        neighbours=sorted(self.samples,key=lambda s:sum(w*(a-b)**2 for w,a,b in zip(weights,features,s[:4])))[:self.k]
        votes=[0.,0.,0.]
        for sample in neighbours:
            d=math.sqrt(sum(w*(a-b)**2 for w,a,b in zip(weights,features,sample[:4])))
            votes[int(sample[4])] += 1.0/(d+0.05)
        return max(range(3),key=lambda i:votes[i])
